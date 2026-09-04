package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Wrapper class for an FTC DcMotorEx providing closed-loop PIDF control,
 * continuous angle wrapping, and automatic 7-segment S-Curve motion profiling.
 * * <p><b>USAGE / PROFILE CONSTRAINTS GUIDE:</b></p>
 * <pre>
 * 1. Initialize constraints in init():
 * motor.setSCurveConstraints(maxVel, maxAccel, maxJerk);
 * * 2. How to calculate baseline constraints:
 * - maxVel   (units/sec)   = (Motor RPM / 60) * TicksPerRev * 0.85
 * - maxAccel (units/sec^2) = maxVel / t_accel  (e.g., t_accel = 0.25s to 0.5s)
 * - maxJerk  (units/sec^3) = maxAccel / t_jerk (e.g., t_jerk  = 0.05s to 0.1s)
 * * 3. Driving the motor in your loop:
 * - Standard call: motor.update(targetPos);
 * -> AUTOMATICALLY uses S-Curve profiling if constraints are set.
 * -> FALLS BACK to direct PID if constraints are unconfigured/zero or profile is null.
 * - Direct call: motor.updateDirect(targetPos, targetVel);
 * -> Bypasses S-Curve profiling and drives raw PID control directly.
 * </pre>
 */
public class ClosedLoopDC {
    /** Telemetry verbosity levels to control telemetry overhead on Driver Station. */
    public enum TelemetryVerbosity {
        /** No telemetry added. */
        DISABLED(0),
        /** Basic position, target, error, and status info only. */
        COMPACT(1),
        /** Standard motor stats, currents, and basic PID gains. */
        STANDARD(2),
        /** Full internal state debugging for PIDF tuning. */
        DEBUG(3);

        private final int level;
        TelemetryVerbosity(int level) { this.level = level; }
        public int getLevel() { return level; }
    }

    private final SafeDcMotor motor;
    private PIDController pid;

    private double maxPower;
    private boolean enabled = true;
    private boolean angleMode = false;
    private boolean angleWrappingEnabled = true;
    private final double ticksPerRev;
    private boolean angleLimitsEnabled = false;
    private boolean largeArcAngleLimit = false;
    private double angleLimitA = -180.0;
    private double angleLimitB = 180.0;
    private double angleLimitArcStart = -180.0;
    private double angleLimitArcLength = 360.0;
    private boolean cosineGravityEnabled = false;
    private double gravityHorizontalTicks = 0.0;
    private double gravityPowerSign = 1.0;

    // S-Curve Motion Profiling
    private final SCurveProfile sCurveProfile;
    private final ElapsedTime profileTimer = new ElapsedTime();
    private boolean profilingActive = false;
    private boolean brakingForReprofile = false;
    private boolean profileCommandInitialized = false;
    private double profileStartPosition = 0;
    private double profileTimeOffset = 0;
    private double activeProfileTarget = 0;
    private double maxVel = 0;
    private double maxAccel = 0;
    private double maxJerk = 0;
    private static final double TARGET_EPSILON = 1e-4;
    private static final double VELOCITY_EPSILON = 1e-3;

    // Telemetry configuration
    private TelemetryVerbosity verbosity = TelemetryVerbosity.STANDARD;
    private double telemetryIntervalSeconds = 0.05; // 20 Hz default rate limit
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private boolean telemetrySnapshotInitialized = false;

    // Telemetry and state tracking variables
    private double lastTarget = 0;
    private double lastCurrent = 0;
    private double lastPidTarget = 0;
    private double lastPidCurrent = 0;
    private double lastPidOutput = 0;
    private double lastEffectiveKg = 0;
    private double lastLimitedTarget = 0;
    private double lastPower = 0;
    private double lastRawPosition = 0;
    private double lastVelocity = 0;
    private double lastTelemetryCurrentAmps = 0;
    private boolean lastTelemetryOverCurrent = false;

    /**
     * Helper method to wrap angles within the range [-180, 180) degrees.
     *
     * @param angle Input angle in degrees.
     * @return Normalized angle in [-180, 180) degrees.
     */
    private double wrapAngle(double angle) {
        angle = (angle + 180) % 360;
        if (angle < 0) {
            angle += 360;
        }
        return angle - 180;
    }

    private double normalizeLimitAngle(double angle) {
        if (!Double.isFinite(angle)) {
            return 0.0;
        }
        double normalized = wrapAngle(angle);
        return normalized <= -180.0 ? 180.0 : normalized;
    }

    private double getPositiveAngleDistance(double from, double to) {
        double distance = normalizeLimitAngle(to) - normalizeLimitAngle(from);
        distance %= 360.0;
        if (distance < 0) {
            distance += 360.0;
        }
        return distance;
    }

    private double getAngleLimitArcEnd() {
        return normalizeLimitAngle(angleLimitArcStart + angleLimitArcLength);
    }

    private boolean isFullAngleLimitArc() {
        return angleLimitArcLength >= 360.0 - TARGET_EPSILON;
    }

    private void updateAngleLimitArc() {
        angleLimitA = normalizeLimitAngle(angleLimitA);
        angleLimitB = normalizeLimitAngle(angleLimitB);

        double forwardDistance = getPositiveAngleDistance(angleLimitA, angleLimitB);
        boolean firstToSecondIsSmallArc = forwardDistance <= 180.0;
        double smallArcStart = firstToSecondIsSmallArc ? angleLimitA : angleLimitB;
        double smallArcLength = firstToSecondIsSmallArc ? forwardDistance : 360.0 - forwardDistance;

        if (largeArcAngleLimit) {
            angleLimitArcStart = normalizeLimitAngle(smallArcStart + smallArcLength);
            angleLimitArcLength = 360.0 - smallArcLength;
        } else {
            angleLimitArcStart = smallArcStart;
            angleLimitArcLength = smallArcLength;
        }
    }

    private boolean isAngleWithinLimits(double angle) {
        if (!angleLimitsEnabled || isFullAngleLimitArc()) {
            return true;
        }
        return getPositiveAngleDistance(angleLimitArcStart, angle) <= angleLimitArcLength + TARGET_EPSILON;
    }

    private double clampAngleToLimits(double angle) {
        double normalized = normalizeLimitAngle(angle);
        if (!angleLimitsEnabled || isFullAngleLimitArc() || isAngleWithinLimits(normalized)) {
            return normalized;
        }

        double arcEnd = getAngleLimitArcEnd();
        double distanceToStart = Math.abs(wrapAngle(normalized - angleLimitArcStart));
        double distanceToEnd = Math.abs(wrapAngle(normalized - arcEnd));
        return distanceToStart <= distanceToEnd ? angleLimitArcStart : arcEnd;
    }

    private double limitTarget(double target) {
        if (!Double.isFinite(target)) {
            return lastTarget;
        }
        if (angleMode && angleWrappingEnabled && angleLimitsEnabled) {
            lastLimitedTarget = clampAngleToLimits(target);
            return lastLimitedTarget;
        }
        lastLimitedTarget = target;
        return target;
    }

    private double getSignedDistance(double from, double to) {
        if (!angleMode) {
            return to - from;
        }
        if (!angleWrappingEnabled) {
            return to - from;
        }
        if (!angleLimitsEnabled || isFullAngleLimitArc()) {
            return wrapAngle(to - from);
        }

        double currentAngle = normalizeLimitAngle(from);
        double targetAngle = clampAngleToLimits(to);
        if (!isAngleWithinLimits(currentAngle)) {
            return wrapAngle(targetAngle - currentAngle);
        }

        return getPositiveAngleDistance(angleLimitArcStart, targetAngle)
                - getPositiveAngleDistance(angleLimitArcStart, currentAngle);
    }

    private double getCurrentVelocity() {
        double velocity = motor.getVelocity();
        if (!Double.isFinite(velocity)) {
            return 0.0;
        }
        if (angleMode && Math.abs(ticksPerRev) > VELOCITY_EPSILON) {
            return velocity * 360.0 / ticksPerRev;
        }
        return velocity;
    }

    private double ticksToUnits(double ticks) {
        if (angleMode && Math.abs(ticksPerRev) > VELOCITY_EPSILON) {
            return ticks * 360.0 / ticksPerRev;
        }
        return ticks;
    }

    private double getArmAngleRadians(double encoderTicks) {
        if (Math.abs(ticksPerRev) <= VELOCITY_EPSILON) {
            return 0.0;
        }
        return ((encoderTicks - gravityHorizontalTicks) / ticksPerRev) * 2.0 * Math.PI;
    }

    private double getEffectiveGravityFeedforward(double encoderTicks) {
        if (pid == null) {
            return 0.0;
        }
        if (!cosineGravityEnabled) {
            return pid.getkG();
        }
        return pid.getkG() * gravityPowerSign * Math.cos(getArmAngleRadians(encoderTicks));
    }

    /**
     * Constructs a ClosedLoopDC motor controller wrapper.
     *
     * @param motor       The DcMotorEx instance to control.
     * @param pid         The PIDController instance.
     * @param maxPower    Maximum motor power limit (0.0 to 1.0).
     * @param ticksPerRev Encoder ticks per revolution of the motor/mechanism.
     */
    public ClosedLoopDC(SafeDcMotor motor, PIDController pid, double maxPower, double ticksPerRev) {
        this.motor = motor;
        this.pid = pid;
        this.maxPower = maxPower;
        this.ticksPerRev = ticksPerRev;
        this.sCurveProfile = new SCurveProfile();
        this.telemetryTimer.reset();
    }

    /**
     * Configures physical constraints for S-Curve profiling.
     * Once set, all calls to update(target) will automatically generate smooth S-curve trajectories.
     *
     * @param maxVel   Maximum velocity (ticks/sec or deg/sec if angleMode is true).
     * @param maxAccel Maximum acceleration (ticks/sec^2 or deg/sec^2).
     * @param maxJerk  Maximum jerk limit (ticks/sec^3 or deg/sec^3).
     */
    public void setSCurveConstraints(double maxVel, double maxAccel, double maxJerk) {
        this.maxVel = Double.isFinite(maxVel) ? Math.abs(maxVel) : 0.0;
        this.maxAccel = Double.isFinite(maxAccel) ? Math.abs(maxAccel) : 0.0;
        this.maxJerk = Double.isFinite(maxJerk) ? Math.abs(maxJerk) : 0.0;
    }

    /**
     * Helper check to verify if valid profile parameters are configured.
     *
     * @return True if velocity, acceleration, and jerk limits are all strictly positive.
     */
    public boolean isProfileConfigured() {
        return sCurveProfile != null && maxVel > 0 && maxAccel > 0 && maxJerk > 0;
    }

    /**
     * Explicitly sets a profiled target. Generates an S-curve profile if constraints are set,
     * otherwise falls back to standard direct PID control.
     *
     * @param targetPosition Desired target position (ticks or degrees).
     */
    public void setProfileTarget(double targetPosition) {
        targetPosition = limitTarget(targetPosition);
        if (!isProfileConfigured()) {
            // Fallback: If unconfigured or null profile, drive direct PID
            profilingActive = false;
            updateDirect(targetPosition, 0);
            return;
        }

        lastTarget = targetPosition;
        profileCommandInitialized = true;
        startDynamicProfile(targetPosition);
    }

    private void startDynamicProfile(double targetPosition) {
        double currentPos = getCurrentPosition();
        double currentVelocity = getCurrentVelocity();

        if (!Double.isFinite(targetPosition) || !Double.isFinite(currentPos) || !Double.isFinite(currentVelocity)) {
            profilingActive = false;
            brakingForReprofile = false;
            return;
        }

        double distance = getSignedDistance(currentPos, targetPosition);
        double absDistance = Math.abs(distance);
        double absVelocity = Math.abs(currentVelocity);

        if (absDistance < TARGET_EPSILON && absVelocity < VELOCITY_EPSILON) {
            profilingActive = false;
            brakingForReprofile = false;
            activeProfileTarget = targetPosition;
            return;
        }

        double stoppingDistance =
                SCurveProfile.calculateStoppingDistance(absVelocity, maxAccel, maxJerk);
        double targetDirection = Math.signum(distance);
        double velocityTowardTarget = currentVelocity * targetDirection;
        boolean movingAway = absVelocity > VELOCITY_EPSILON && velocityTowardTarget <= 0;
        boolean overSpeed = absVelocity > maxVel;
        boolean cannotStopBeforeTarget =
                absVelocity > VELOCITY_EPSILON
                        && stoppingDistance + TARGET_EPSILON >= absDistance;

        if (movingAway || overSpeed || cannotStopBeforeTarget) {
            double brakeDirection = Math.signum(currentVelocity);
            if (brakeDirection == 0) {
                brakeDirection = targetDirection == 0 ? 1.0 : targetDirection;
            }
            double brakeDistance = Math.max(stoppingDistance, TARGET_EPSILON);
            double brakeTarget = currentPos + (brakeDirection * brakeDistance);
            generateProfileFromMotion(brakeTarget, currentPos, currentVelocity);
            brakingForReprofile = profilingActive;
            return;
        }

        generateProfileFromMotion(targetPosition, currentPos, currentVelocity);
        brakingForReprofile = false;
    }

    private void generateProfileFromMotion(double targetPosition, double currentPos, double currentVelocity) {
        double distance = getSignedDistance(currentPos, targetPosition);
        double direction = Math.signum(distance);
        double absDistance = Math.abs(distance);

        if (direction == 0 || absDistance < TARGET_EPSILON) {
            profilingActive = false;
            activeProfileTarget = targetPosition;
            return;
        }

        double signedVelocity = currentVelocity * direction;
        double profileVelocity = Math.max(0.0, Math.min(Math.abs(signedVelocity), maxVel));

        if (signedVelocity <= VELOCITY_EPSILON || profileVelocity < VELOCITY_EPSILON) {
            profileStartPosition = currentPos;
            profileTimeOffset = 0.0;
            activeProfileTarget = targetPosition;
            sCurveProfile.generate(maxVel, maxAccel, maxJerk, distance);
            profileTimer.reset();
            profilingActive = sCurveProfile.getTotalTime() > 0;
            return;
        }

        double accelerationDistance =
                SCurveProfile.calculateAccelerationDistance(profileVelocity, maxAccel, maxJerk);
        double totalProfileDistance = absDistance + accelerationDistance;

        if (!Double.isFinite(totalProfileDistance) || totalProfileDistance < TARGET_EPSILON) {
            profilingActive = false;
            activeProfileTarget = targetPosition;
            return;
        }

        profileStartPosition = currentPos - (direction * accelerationDistance);
        profileTimeOffset =
                Math.min(
                        SCurveProfile.calculateAccelerationTime(profileVelocity, maxAccel, maxJerk),
                        Double.MAX_VALUE);
        activeProfileTarget = targetPosition;
        sCurveProfile.generate(maxVel, maxAccel, maxJerk, direction * totalProfileDistance);
        profileTimeOffset = Math.min(profileTimeOffset, sCurveProfile.getTotalTime());
        profileTimer.reset();
        profilingActive = sCurveProfile.getTotalTime() > 0;
    }

    /**
     * Standard update call. Defaults to running/updating the S-Curve profile if configured.
     * If unconfigured or profile completed, smoothly holds position via PID controller.
     *
     * @param target Desired final position (encoder ticks or degrees).
     */
    public void update(double target) {
        target = limitTarget(target);
        if (!isProfileConfigured()) {
            profilingActive = false;
            brakingForReprofile = false;
            updateDirect(target, 0);
            return;
        }

        boolean targetChanged =
                !profileCommandInitialized || Math.abs(getSignedDistance(lastTarget, target)) > TARGET_EPSILON;

        if (targetChanged) {
            lastTarget = target;
            profileCommandInitialized = true;
            startDynamicProfile(target);
        }

        if (profilingActive && isProfileConfigured()) {
            double t = profileTimer.seconds() + profileTimeOffset;
            SCurveProfile.ProfileState state = sCurveProfile.calculate(t);

            double currentProfileTargetPos = profileStartPosition + state.position;
            double targetVelocity = state.velocity;

            updateDirectInternal(currentProfileTargetPos, targetVelocity, false);

            if (t >= sCurveProfile.getTotalTime()) {
                profilingActive = false;
                if (brakingForReprofile) {
                    brakingForReprofile = false;
                    startDynamicProfile(lastTarget);
                }
            }
        } else {
            // Direct PID fallback if profile is null/unconfigured or finished
            updateDirectInternal(target, 0, false);
        }
    }

    /**
     * Updates motor power directly using target position error and target velocity feedforward.
     *
     * @param target         Target position in ticks/degrees.
     * @param targetVelocity Target velocity in units/sec.
     */
    public void updateDirect(double target, double targetVelocity) {
        target = limitTarget(target);
        updateDirectInternal(target, targetVelocity, true);
    }

    private void updateDirectInternal(double target, double targetVelocity, boolean updateRequestedTarget) {
        if (updateRequestedTarget) {
            lastTarget = target;
            activeProfileTarget = target;
            profileCommandInitialized = true;
            profilingActive = false;
            brakingForReprofile = false;
        }

        lastRawPosition = motor.getCurrentPosition();
        lastVelocity = getCurrentVelocity();

        if (!enabled || pid == null) {
            motor.setPower(0);
            lastCurrent = ticksToUnits(lastRawPosition);
            lastPidTarget = target;
            lastPidCurrent = lastCurrent;
            lastPidOutput = 0;
            lastPower = 0;
            motor.update();
            return;
        }

        double currentPos = lastRawPosition;
        double pidTarget;
        double pidCurrent;

        if (angleMode) {
            double currentAngle = ticksToUnits(currentPos);
            pidTarget = currentAngle + getSignedDistance(currentAngle, target);
            pidCurrent = currentAngle;
        } else {
            pidTarget = target;
            pidCurrent = currentPos;
        }

        lastEffectiveKg = getEffectiveGravityFeedforward(lastRawPosition);

        double power = pid.update(pidTarget, pidCurrent, targetVelocity, lastEffectiveKg);
        lastPidOutput = power;

        // Clamp output power
        power = Math.max(Math.min(power, maxPower), -maxPower);

        lastCurrent = pidCurrent;
        lastPidTarget = pidTarget;
        lastPidCurrent = pidCurrent;
        lastPower = power;

        if (enabled) {
            motor.setPower(power);
        }
        motor.update();
    }

    /** @return True if an S-Curve motion profile is actively being executed. */
    public boolean isProfiling() {
        return profilingActive;
    }

    /**
     * Initializes motor modes and resets controller internal state.
     *
     * @param isAuto True if running in Autonomous.
     */
    public void init(boolean isAuto) {
        if (isAuto) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        reset();
    }

    /**
     * Sets max motor power limit.
     *
     * @param maxPower Power cap between 0.0 and 1.0.
     */
    public void setMaxPower(double maxPower) {
        this.maxPower = Math.abs(maxPower);
    }

    /**
     * Enables arm-style gravity feedforward using kG * cos(theta).
     *
     * @param horizontalTicks Encoder ticks where the arm is horizontal.
     * @param gravityPowerSign Sign of positive gravity compensation power.
     */
    public void enableCosineGravityFeedforward(double horizontalTicks, double gravityPowerSign) {
        this.cosineGravityEnabled = true;
        this.gravityHorizontalTicks = horizontalTicks;
        this.gravityPowerSign = Math.signum(gravityPowerSign);
        if (this.gravityPowerSign == 0) {
            this.gravityPowerSign = 1.0;
        }
    }

    /** Disables dynamic cosine gravity and returns to constant PIDController kG. */
    public void disableCosineGravityFeedforward() {
        this.cosineGravityEnabled = false;
    }

    /** @return True if kG is scaled by cos(theta). */
    public boolean isCosineGravityEnabled() {
        return cosineGravityEnabled;
    }

    /** @return Encoder ticks where the arm is horizontal. */
    public double getGravityHorizontalTicks() {
        return gravityHorizontalTicks;
    }

    /** @return Sign applied to cosine gravity feedforward. */
    public double getGravityPowerSign() {
        return gravityPowerSign;
    }

    /** @return Last gravity feedforward value passed into PIDController. */
    public double getLastEffectiveKg() {
        return lastEffectiveKg;
    }

    /**
     * Sets the PIDController gravity feedforward value used when cosine gravity is disabled.
     *
     * @param gravityFeedforward Gravity feedforward motor power for this update.
     */
    public void setGravityFeedforward(double gravityFeedforward) {
        if (pid != null && Double.isFinite(gravityFeedforward)) {
            pid.setkG(gravityFeedforward);
        }
    }

    /**
     * Sets telemetry verbosity level.
     *
     * @param verbosity Desired TelemetryVerbosity level.
     */
    public void setTelemetryVerbosity(TelemetryVerbosity verbosity) {
        this.verbosity = verbosity;
    }

    /**
     * Sets telemetry verbosity level by integer code.
     *
     * @param level Integer level 0 to 3.
     */
    public void setTelemetryVerbosity(int level) {
        switch (level) {
            case 0: setTelemetryVerbosity(TelemetryVerbosity.DISABLED); break;
            case 1: setTelemetryVerbosity(TelemetryVerbosity.COMPACT); break;
            case 3: setTelemetryVerbosity(TelemetryVerbosity.DEBUG); break;
            default: setTelemetryVerbosity(TelemetryVerbosity.STANDARD); break;
        }
    }

    /**
     * Sets telemetry update rate limit.
     *
     * @param intervalSeconds Update interval in seconds.
     */
    public void setTelemetryInterval(double intervalSeconds) {
        this.telemetryIntervalSeconds = Math.max(0, intervalSeconds);
    }

    /** @return Current position in ticks or degrees depending on angleMode. */
    public double getCurrentPosition() {
        return ticksToUnits(motor.getCurrentPosition());
    }

    /** @return Last sampled current position in ticks/degrees. */
    public double getLastCurrent() {
        return lastCurrent;
    }

    /** @return Last raw encoder position in ticks. */
    public double getLastRawPosition() {
        return lastRawPosition;
    }

    /** @return Last sampled mechanism velocity in ticks/sec or deg/sec. */
    public double getLastVelocity() {
        return lastVelocity;
    }

    /** @return True if PID position error is within strict deadband. */
    public boolean isOnTarget() {
        return pid != null && pid.isOnTarget();
    }

    /** @return True if position error is within settled tolerance. */
    public boolean isSettled() {
        return pid != null && pid.isSettled();
    }

    /** @return Target error taking angle wrapping into account. */
    public double getTargetError() {
        double current = getCurrentPosition();
        if (angleMode) {
            return getSignedDistance(current, lastTarget);
        }
        return lastTarget - current;
    }

    /** @return Strict deadband from PID controller. */
    public double getPositionTolerance() {
        return (pid == null) ? 0 : pid.getDeadband();
    }

    /** @return Settled tolerance from PID controller. */
    public double getSettledTolerance() {
        return (pid == null) ? 0 : pid.getSettledDeadband();
    }

    /** @return Last requested user target. */
    public double getLastTarget() {
        return lastTarget;
    }

    /** @return Last unwrapped target passed to internal PID. */
    public double getLastPidTarget() {
        return lastPidTarget;
    }

    /** @return Last current value passed to internal PID. */
    public double getLastPidCurrent() {
        return lastPidCurrent;
    }

    /** @return Last unclamped PIDF output. */
    public double getLastPidOutput() {
        return lastPidOutput;
    }

    /** @return Last clamped motor power command. */
    public double getLastPower() {
        return lastPower;
    }

    /** @return Current max output power limit. */
    public double getMaxPower() {
        return maxPower;
    }

    /** @return Current active profile target. */
    public double getActiveProfileTarget() {
        return activeProfileTarget;
    }

    /** @return Current profile elapsed time including any dynamic reprofile offset. */
    public double getProfileTime() {
        return profileTimer.seconds() + profileTimeOffset;
    }

    /** @return Total time of the active/generated S-curve profile. */
    public double getProfileTotalTime() {
        return sCurveProfile.getTotalTime();
    }

    /** @return True if currently braking before generating a new dynamic profile. */
    public boolean isBrakingForReprofile() {
        return brakingForReprofile;
    }

    /** @return Arm gravity angle in degrees from the configured horizontal tick position. */
    public double getGravityAngleDegrees() {
        return Math.toDegrees(getArmAngleRadians(lastRawPosition));
    }

    /**
     * Enables or disables angle mode.
     *
     * @param angleMode True for continuous angle in degrees.
     */
    public void setAngleMode(boolean angleMode) {
        setAngleMode(angleMode, true);
    }

    /**
     * Enables angle mode and selects whether angle targets should wrap through (-180, 180).
     *
     * @param wrapAngles True for shortest-path wrapped angles, false for continuous multi-turn degrees.
     */
    public void setAngleMode(boolean angleMode, boolean wrapAngles) {
        boolean changed = this.angleMode != angleMode || this.angleWrappingEnabled != wrapAngles;
        this.angleMode = angleMode;
        this.angleWrappingEnabled = wrapAngles;
        if (!wrapAngles) {
            angleLimitsEnabled = false;
        }
        if (changed) {
            reset();
        }
    }

    /** Enables degree units with shortest-path wrapping. */
    public void enableWrappedAngleMode() {
        setAngleMode(true, true);
    }

    /** Enables degree units without wrapping, useful for elevators or multi-turn joints. */
    public void enableNonWrappedAngleMode() {
        setAngleMode(true, false);
    }

    /**
     * Enables or disables shortest-path angle wrapping while keeping angle mode state unchanged.
     *
     * @param enabled True to wrap angle errors, false to use continuous degree error.
     */
    public void setAngleWrappingEnabled(boolean enabled) {
        if (angleWrappingEnabled != enabled) {
            angleWrappingEnabled = enabled;
            if (!enabled) {
                angleLimitsEnabled = false;
            }
            reset();
        }
    }

    /** @return True if angle mode uses shortest-path wrapping. */
    public boolean isAngleWrappingEnabled() {
        return angleWrappingEnabled;
    }

    /**
     * Enables angular travel limits using either the smaller or larger arc between two endpoints.
     *
     * @param firstLimitDegrees First endpoint, normalized internally to (-180, 180].
     * @param secondLimitDegrees Second endpoint, normalized internally to (-180, 180].
     * @param useLargeArc True to allow the larger arc between endpoints, false for the smaller arc.
     */
    public void setAngleLimits(double firstLimitDegrees, double secondLimitDegrees, boolean useLargeArc) {
        if (!angleWrappingEnabled) {
            angleLimitsEnabled = false;
            return;
        }
        angleLimitsEnabled = true;
        largeArcAngleLimit = useLargeArc;
        angleLimitA = firstLimitDegrees;
        angleLimitB = secondLimitDegrees;
        updateAngleLimitArc();
        lastTarget = limitTarget(lastTarget);
        activeProfileTarget = limitTarget(activeProfileTarget);
    }

    /** Enables limits on the smaller arc between the two endpoint angles. */
    public void enableSmallArcAngleLimits(double firstLimitDegrees, double secondLimitDegrees) {
        setAngleLimits(firstLimitDegrees, secondLimitDegrees, false);
    }

    /** Enables limits on the larger arc between the two endpoint angles. */
    public void enableLargeArcAngleLimits(double firstLimitDegrees, double secondLimitDegrees) {
        setAngleLimits(firstLimitDegrees, secondLimitDegrees, true);
    }

    /** Disables angular travel limits. */
    public void disableAngleLimits() {
        angleLimitsEnabled = false;
    }

    /** @return True if angle targets are clamped to configured arc limits. */
    public boolean areAngleLimitsEnabled() {
        return angleLimitsEnabled;
    }

    /** @return True if configured angle limits use the larger arc between endpoints. */
    public boolean isLargeArcAngleLimit() {
        return largeArcAngleLimit;
    }

    /** @return First configured angle limit endpoint in degrees, normalized to (-180, 180]. */
    public double getAngleLimitA() {
        return angleLimitA;
    }

    /** @return Second configured angle limit endpoint in degrees, normalized to (-180, 180]. */
    public double getAngleLimitB() {
        return angleLimitB;
    }

    /** @return Start of the currently allowed limit arc in degrees. */
    public double getAngleLimitArcStart() {
        return angleLimitArcStart;
    }

    /** @return End of the currently allowed limit arc in degrees. */
    public double getAngleLimitArcEndDegrees() {
        return getAngleLimitArcEnd();
    }

    /** @return Length of the currently allowed limit arc in degrees. */
    public double getAngleLimitArcLength() {
        return angleLimitArcLength;
    }

    /** @return Last target after angular limit clamping. */
    public double getLastLimitedTarget() {
        return lastLimitedTarget;
    }

    /** Enables or disables motor power output. */
    public void enable(boolean enabled) {
        if (enabled && !this.enabled) {
            reset();
        }
        this.enabled = enabled;
    }

    /** @return True if controller output is enabled. */
    public boolean isEnabled() {
        return enabled;
    }

    /** Resets controller internal timers, profiles, and PID state. */
    public void reset() {
        if (pid != null) {
            pid.reset();
        }
        profilingActive = false;
        brakingForReprofile = false;
        profileCommandInitialized = false;
        profileTimeOffset = 0.0;
        activeProfileTarget = lastTarget;
        telemetryTimer.reset();
        telemetrySnapshotInitialized = false;
    }

    /** Replaces current PIDController instance. */
    public void setPIDController(PIDController pid) {
        this.pid = pid;
    }

    /** Appends diagnostic output to telemetry based on rate limits and verbosity settings. */
    public void appendTelemetry(Telemetry telemetry, String name) {
        if (verbosity == TelemetryVerbosity.DISABLED) return;

        if (!telemetrySnapshotInitialized || telemetryTimer.seconds() >= telemetryIntervalSeconds) {
            telemetrySnapshotInitialized = true;
            telemetryTimer.reset();
            lastTelemetryCurrentAmps = motor.getCurrent(CurrentUnit.AMPS);
            lastTelemetryOverCurrent = motor.isOverCurrent();
        }

        int level = verbosity.getLevel();
        telemetry.addLine("  " + name);

        if (level >= 1) {
            telemetry.addData(name + " Pos/Tgt", "%.1f / %.1f", getCurrentPosition(), lastTarget);
            telemetry.addData(name + " Error", "%.2f", getTargetError());
            telemetry.addData(name + " Power", "%.2f", lastPower);
            telemetry.addData(name + " Status", "OnTgt: %b | Settled: %b | Profiled: %b", isOnTarget(), isSettled(), profilingActive);
        }

        if (level >= 2) {
            telemetry.addData(name + " Enabled", enabled);
            telemetry.addData(name + " Angle Mode", angleMode);
            telemetry.addData(name + " Angle Wrap", angleWrappingEnabled);
            telemetry.addData(name + " Angle Limits", "%b largeArc %b", angleLimitsEnabled, largeArcAngleLimit);
            telemetry.addData(name + " Cosine kG", cosineGravityEnabled);
            telemetry.addData(name + " Effective kG", "%.4f", lastEffectiveKg);
            telemetry.addData(name + " Raw Pos", "%.0f ticks", lastRawPosition);
            telemetry.addData(name + " Velocity", "%.2f units/s", lastVelocity);
            telemetry.addData(name + " Current", "%.2f A", lastTelemetryCurrentAmps);
            telemetry.addData(name + " Over Current", lastTelemetryOverCurrent);

            if (pid != null) {
                telemetry.addData(name + " PID Gains", "P %.4f I %.4f D %.4f", pid.getkP(), pid.getkI(), pid.getkD());
                telemetry.addData(name + " FF Gains", "S %.4f V %.4f A %.4f", pid.getkS(), pid.getkV(), pid.getkA());
            }
        }

        if (level >= 3) {
            telemetry.addData(name + " Max Power", "%.3f", maxPower);
            telemetry.addData(name + " PID Target", "%.2f", lastPidTarget);
            telemetry.addData(name + " PID Current", "%.2f", lastPidCurrent);
            telemetry.addData(name + " PID Output", "%.3f", lastPidOutput);

            if (pid != null) {
                telemetry.addData(name + " PID Internal State", "err %.2f sum %.3f dt %.3f",
                        pid.getLastError(), pid.getErrorSum(), pid.getLastDt());
                telemetry.addData(name + " Integral Delta", "%.3f", pid.getLastIntegralDelta());
                telemetry.addData(name + " PID Resets", pid.getResetCount());
                telemetry.addData(name + " PID Tolerances", "deadband %.2f settled %.2f integral %.2f..%.2f",
                        pid.getDeadband(), pid.getSettledDeadband(), pid.getIntegralMin(), pid.getIntegralMax());
                telemetry.addData(name + " Profile Target", "%.2f", activeProfileTarget);
                telemetry.addData(name + " Profile Time", "%.3f / %.3f",
                        profileTimer.seconds() + profileTimeOffset, sCurveProfile.getTotalTime());
                telemetry.addData(name + " Reprofile Brake", brakingForReprofile);
                telemetry.addData(name + " Gravity Angle", "%.2f deg",
                        Math.toDegrees(getArmAngleRadians(lastRawPosition)));
                telemetry.addData(name + " Horizontal Ticks", "%.1f", gravityHorizontalTicks);
                telemetry.addData(name + " Limited Target", "%.2f", lastLimitedTarget);
                telemetry.addData(name + " Angle Limit Endpoints", "%.1f / %.1f", angleLimitA, angleLimitB);
                telemetry.addData(name + " Angle Limit Arc", "%.1f -> %.1f len %.1f",
                        angleLimitArcStart, getAngleLimitArcEnd(), angleLimitArcLength);
            } else {
                telemetry.addData(name + " PID", "null");
            }
        }
    }
}
