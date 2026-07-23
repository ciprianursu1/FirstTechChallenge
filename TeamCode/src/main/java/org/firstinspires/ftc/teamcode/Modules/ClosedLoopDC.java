package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Wrapper class for an FTC DcMotorEx providing closed-loop PIDF control
 * supporting raw encoder ticks or continuous angle modes.
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

    private final DcMotorEx motor;
    private PIDController pid;

    private double maxPower;
    private boolean enabled = true;
    private boolean angleMode = false;
    private final double ticksPerRev;

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

    /**
     * Constructs a ClosedLoopDC motor controller wrapper.
     *
     * @param motor       The DcMotorEx instance to control.
     * @param pid         The PIDController instance.
     * @param maxPower    Maximum motor power limit (0.0 to 1.0).
     * @param ticksPerRev Encoder ticks per revolution of the motor/mechanism.
     */
    public ClosedLoopDC(DcMotorEx motor, PIDController pid, double maxPower, double ticksPerRev) {
        this.motor = motor;
        this.pid = pid;
        this.maxPower = maxPower;
        this.ticksPerRev = ticksPerRev;
        this.telemetryTimer.reset();
    }

    /**
     * Initializes the motor modes and resets controller internal state.
     *
     * @param isAuto True if running in Autonomous (resets encoder position to 0).
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
     * Sets the maximum power limit for the motor.
     *
     * @param maxPower Maximum power scalar between 0.0 and 1.0.
     */
    public void setMaxPower(double maxPower) {
        this.maxPower = Math.abs(maxPower);
    }

    /**
     * Sets the telemetry verbosity output level.
     *
     * @param verbosity Desired TelemetryVerbosity level (DISABLED, COMPACT, STANDARD, DEBUG).
     */
    public void setTelemetryVerbosity(TelemetryVerbosity verbosity) {
        this.verbosity = verbosity;
    }

    /**
     * Sets the telemetry verbosity level by integer code (0 = DISABLED, 1 = COMPACT, 2 = STANDARD, 3 = DEBUG).
     *
     * @param level Integer verbosity level 0 to 3.
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
     * Sets the minimum time interval between telemetry updates to avoid saturating telemetry bandwidth.
     *
     * @param intervalSeconds Minimum time between updates in seconds (e.g. 0.05 for 20 Hz).
     */
    public void setTelemetryInterval(double intervalSeconds) {
        this.telemetryIntervalSeconds = Math.max(0, intervalSeconds);
    }

    /**
     * Gets the current position of the motor in ticks or degrees depending on mode.
     *
     * @return Current motor position in degrees (if angleMode) or ticks.
     */
    public double getCurrentPosition() {
        if (angleMode) {
            return motor.getCurrentPosition() * 360.0 / ticksPerRev;
        } else {
            return motor.getCurrentPosition();
        }
    }

    /**
     * Checks if the position error is within the strict position deadband.
     *
     * @return True if output power is zeroed due to being on target.
     */
    public boolean isOnTarget() {
        return pid != null && pid.isOnTarget();
    }

    /**
     * Checks if the position error is within the settled acceptance deadband.
     * Useful for allowing state machines to proceed while PID refines target position.
     *
     * @return True if mechanism is within settled tolerance.
     */
    public boolean isSettled() {
        return pid != null && pid.isSettled();
    }

    /**
     * Calculates current target error taking angle wrapping into account if enabled.
     *
     * @return Target position minus current position.
     */
    public double getTargetError() {
        double current = getCurrentPosition();
        if (angleMode) {
            return wrapAngle(lastTarget - current);
        }
        return lastTarget - current;
    }

    /**
     * Gets the strict position deadband from the PID controller.
     *
     * @return Position tolerance in ticks or degrees.
     */
    public double getPositionTolerance() {
        if (pid == null) return 0;
        return pid.getDeadband();
    }

    /**
     * Gets the settled acceptance deadband from the PID controller.
     *
     * @return Settled tolerance in ticks or degrees.
     */
    public double getSettledTolerance() {
        if (pid == null) return 0;
        return pid.getSettledDeadband();
    }

    /** @return Last raw user target position passed to update. */
    public double getLastTarget() {
        return lastTarget;
    }

    /** @return Last unwrapped target position passed to the PID controller. */
    public double getLastPidTarget() {
        return lastPidTarget;
    }

    /**
     * Enables or disables angle mode (degrees with wrapping vs raw encoder ticks).
     *
     * @param angleMode True to enable angle mode in degrees.
     */
    public void setAngleMode(boolean angleMode) {
        if (this.angleMode != angleMode) {
            this.angleMode = angleMode;
            reset();
        }
    }

    /**
     * Updates the motor power using position feedback (assuming target velocity = 0).
     *
     * @param target Desired position in encoder ticks (or degrees if angleMode is true).
     */
    public void update(double target) {
        update(target, 0);
    }

    /**
     * Updates the motor power using position feedback and target velocity feedforward.
     *
     * @param target         Desired position in encoder ticks (or degrees if angleMode is true).
     * @param targetVelocity Planned target velocity in ticks/sec (or deg/sec if angleMode is true).
     */
    public void update(double target, double targetVelocity) {
        lastTarget = target;
        lastRawPosition = motor.getCurrentPosition();
        lastVelocity = motor.getVelocity();

        if (!enabled || pid == null) {
            motor.setPower(0);
            lastCurrent = angleMode ? lastRawPosition * 360.0 / ticksPerRev : lastRawPosition;
            lastPidTarget = target;
            lastPidCurrent = lastCurrent;
            lastPidOutput = 0;
            lastPower = 0;
            return;
        }

        double currentPos = lastRawPosition;
        double pidTarget;
        double pidCurrent;

        if (angleMode) {
            double currentAngle = currentPos * 360.0 / ticksPerRev;
            pidTarget = currentAngle + wrapAngle(target - currentAngle);
            pidCurrent = currentAngle;
        } else {
            pidTarget = target;
            pidCurrent = currentPos;
        }

        double power = pid.update(pidTarget, pidCurrent, targetVelocity);
        lastPidOutput = power;

        // Clamp output to configured max motor power
        power = Math.max(Math.min(power, maxPower), -maxPower);

        lastCurrent = pidCurrent;
        lastPidTarget = pidTarget;
        lastPidCurrent = pidCurrent;
        lastPower = power;

        if (enabled) {
            motor.setPower(power);
        }
    }

    /**
     * Enables or disables closed-loop motor control.
     *
     * @param enabled True to enable power output, false to disable and stop motor.
     */
    public void enable(boolean enabled) {
        if (enabled && !this.enabled) {
            reset();
        }
        this.enabled = enabled;
    }

    /** @return True if closed-loop control is active. */
    public boolean isEnabled() {
        return enabled;
    }

    /** Resets controller internal state and timer. */
    public void reset() {
        if (pid != null) {
            pid.reset();
        }
        telemetryTimer.reset();
    }

    /**
     * Sets or replaces the PIDController instance.
     *
     * @param pid New PIDController instance.
     */
    public void setPIDController(PIDController pid) {
        this.pid = pid;
    }

    /**
     * Appends diagnostic and tuning data to FTC Telemetry output governed by verbosity level
     * and interval timing rate limits.
     *
     * @param telemetry FTC Telemetry object.
     * @param name      Identifier prefix for display formatting.
     */
    public void appendTelemetry(Telemetry telemetry, String name) {
        // Level 0: Disabled or rate-limited
        if (verbosity == TelemetryVerbosity.DISABLED) {
            return;
        }

        if (telemetryTimer.seconds() < telemetryIntervalSeconds) {
            return;
        }
        telemetryTimer.reset();

        int level = verbosity.getLevel();

        telemetry.addLine("  " + name);

        // Level 1: COMPACT (Minimal output for driver control)
        if (level >= 1) {
            telemetry.addData(name + " Pos/Tgt", "%.1f / %.1f", getCurrentPosition(), lastTarget);
            telemetry.addData(name + " Error", "%.2f", getTargetError());
            telemetry.addData(name + " Power", "%.2f", lastPower);
            telemetry.addData(name + " Status", "OnTgt: %b | Settled: %b", isOnTarget(), isSettled());
        }

        // Level 2: STANDARD (Standard mechanical readout)
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

        // Level 3: DEBUG (Full state analysis for deep PID tuning)
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
            } else {
                telemetry.addData(name + " PID", "null");
            }
        }
    }
}