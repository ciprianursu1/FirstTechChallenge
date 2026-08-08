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
    private final double ticksPerRev;

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

    // Telemetry and state tracking variables
    private double lastTarget = 0;
    private double lastCurrent = 0;
    private double lastPidTarget = 0;
    private double lastPidCurrent = 0;
    private double lastPidOutput = 0;
    private double lastPower = 0;
    private double lastRawPosition = 0;
    private double lastVelocity = 0;

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

    private double getSignedDistance(double from, double to) {
        double distance = to - from;
        return angleMode ? wrapAngle(distance) : distance;
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
            pidTarget = currentAngle + wrapAngle(target - currentAngle);
            pidCurrent = currentAngle;
        } else {
            pidTarget = target;
            pidCurrent = currentPos;
        }

        double power = pid.update(pidTarget, pidCurrent, targetVelocity);
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
            return wrapAngle(lastTarget - current);
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

    /**
     * Enables or disables angle mode.
     *
     * @param angleMode True for continuous angle in degrees.
     */
    public void setAngleMode(boolean angleMode) {
        if (this.angleMode != angleMode) {
            this.angleMode = angleMode;
            reset();
        }
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
    }

    /** Replaces current PIDController instance. */
    public void setPIDController(PIDController pid) {
        this.pid = pid;
    }

    /** Appends diagnostic output to telemetry based on rate limits and verbosity settings. */
    public void appendTelemetry(Telemetry telemetry, String name) {
        if (verbosity == TelemetryVerbosity.DISABLED) return;
        if (telemetryTimer.seconds() < telemetryIntervalSeconds) return;
        telemetryTimer.reset();

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
            telemetry.addData(name + " Raw Pos", "%.0f ticks", lastRawPosition);
            telemetry.addData(name + " Velocity", "%.2f units/s", lastVelocity);
            telemetry.addData(name + " Current", "%.2f A", motor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData(name + " Over Current", motor.isOverCurrent());

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
            } else {
                telemetry.addData(name + " PID", "null");
            }
        }
    }
}
