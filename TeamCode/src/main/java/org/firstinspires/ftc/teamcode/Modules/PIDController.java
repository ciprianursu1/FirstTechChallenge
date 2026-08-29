package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Custom PIDF Controller implementation supporting Feedback (PID)
 * and Feedforward (kS, kV, kA, kG) control for FTC robotics mechanisms.
 */
public class PIDController {
    private static final double ZERO = 1e-6;

    // Gains
    private double kP, kI, kD;
    private double kS, kV, kA, kG;

    // State Tracking
    private double errorSum = 0;
    private double lastMeasurement = 0;
    private double lastError = 0;
    private double lastTargetVelocity = 0;

    private double output = 0;
    private double lastDt = 0;
    private double lastMeasuredDt = 0;
    private double integralMax = 1.0, integralMin = -1.0;

    // Dual Deadbands
    private double deadband = 1.0;        // Target position deadband (stops feedback correction)
    private double settledDeadband = 5.0; // Acceptance deadband (allows state machine to move on)

    private double lastIntegralDelta = 0;
    private int resetCount = 0;
    private boolean resetIntegralOnSignChange = false;
    private boolean holdFeedforwardInDeadband = false;
    private boolean isFirstRun = true;

    private final ElapsedTime timer = new ElapsedTime();

    /**
     * Constructs a new PIDController with standard feedback and motion feedforward gains.
     *
     * @param p Proportional gain (kP)
     * @param i Integral gain (kI)
     * @param d Derivative gain (kD)
     * @param s Static friction feedforward gain (kS)
     * @param v Velocity feedforward gain (kV)
     * @param a Acceleration feedforward gain (kA)
     */
    public PIDController(double p, double i, double d, double s, double v, double a) {
        this(p, i, d, s, v, a, 0.0);
    }

    /**
     * Constructs a new PIDController with feedback and full feedforward (including gravity feedforward).
     *
     * @param p Proportional gain (kP)
     * @param i Integral gain (kI)
     * @param d Derivative gain (kD)
     * @param s Static friction feedforward gain (kS)
     * @param v Velocity feedforward gain (kV)
     * @param a Acceleration feedforward gain (kA)
     * @param g Gravity feedforward gain (kG)
     */
    public PIDController(double p, double i, double d, double s, double v, double a, double g) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
        this.kS = s;
        this.kV = v;
        this.kA = a;
        this.kG = g;
        reset();
    }

    /**
     * Updates the PID controller output for position-only control with constant gravity compensation.
     * Target velocity is assumed to be zero.
     *
     * @param targetPosition  Desired target position in encoder ticks/units.
     * @param currentPosition Current measured position in encoder ticks/units.
     * @return Motor power output constrained between -1.0 and 1.0.
     */
    public double update(double targetPosition, double currentPosition) {
        return update(targetPosition, currentPosition, 0, kG);
    }

    /**
     * Updates the PID controller output using feedback (PID) and motion feedforward (kS, kV, kA),
     * assuming constant gravity feedforward (e.g. vertical elevators).
     *
     * @param targetPosition  Desired target position in encoder ticks/units.
     * @param currentPosition Current measured position in encoder ticks/units.
     * @param targetVelocity  Planned velocity at this timestep in units/sec.
     * @return Motor power output constrained between -1.0 and 1.0.
     */
    public double update(double targetPosition, double currentPosition, double targetVelocity) {
        return update(targetPosition, currentPosition, targetVelocity, kG);
    }

    /**
     * Updates the PID controller output using feedback (PID) and feedforward (kS, kV, kA, kG).
     * Allows passing dynamic gravity feedforward (e.g., kG * Math.cos(angle) for pivoting arms).
     *
     * @param targetPosition  Desired target position in encoder ticks/units.
     * @param currentPosition Current measured position in encoder ticks/units.
     * @param targetVelocity  Planned velocity at this timestep in units/sec.
     * @param effectiveKg     Dynamic gravity term for this cycle (e.g. kG or kG * Math.cos(angle)).
     * @return Motor power output constrained between -1.0 and 1.0.
     */
    public double update(double targetPosition, double currentPosition, double targetVelocity, double effectiveKg) {
        double measuredDt = timer.seconds();

        if (measuredDt < ZERO) {
            return output;
        }

        timer.reset();
        lastMeasuredDt = measuredDt;
        lastDt = measuredDt;

        double error = targetPosition - currentPosition;
        double targetAccel = (targetVelocity - lastTargetVelocity) / measuredDt;
        lastTargetVelocity = targetVelocity;

        // Position Deadband: Stop feedback correction when strictly inside the target tolerance.
        if (Math.abs(error) < deadband) {
            lastIntegralDelta = -errorSum;
            errorSum = 0;
            lastError = error;
            lastMeasurement = currentPosition;
            output =
                    holdFeedforwardInDeadband
                            ? calculateFeedforward(targetVelocity, targetAccel, effectiveKg, 0)
                            : 0;
            output = Math.max(Math.min(output, 1.0), -1.0);
            return output;
        }

        // Integral anti-windup on sign change
        if (resetIntegralOnSignChange && Math.signum(error) != Math.signum(lastError) && lastError != 0) {
            errorSum = 0;
        }

        // Derivative calculated on measurement to avoid derivative kick on target jumps
        double derivative = 0;
        if (!isFirstRun) {
            derivative = (currentPosition - lastMeasurement) / measuredDt;
        } else {
            isFirstRun = false;
        }

        lastError = error;
        lastMeasurement = currentPosition;

        // Integral calculation with clamping limits
        double unclampedErrorSum = errorSum + error * measuredDt;
        errorSum = Math.max(Math.min(unclampedErrorSum, integralMax), integralMin);
        lastIntegralDelta = errorSum - (unclampedErrorSum - error * measuredDt);

        // --- 1. Feedback (PID) ---
        double pidOutput = (kP * error) + (kI * errorSum) - (kD * derivative);

        // --- 2. Feedforward (kS, kV, kA, kG) ---
        double ffOutput = calculateFeedforward(targetVelocity, targetAccel, effectiveKg, error);

        // Combine and bound output
        output = pidOutput + ffOutput;
        output = Math.max(Math.min(output, 1.0), -1.0);

        return output;
    }

    private double calculateFeedforward(
            double targetVelocity,
            double targetAccel,
            double effectiveKg,
            double error) {
        double staticFriction = kS * Math.signum(targetVelocity);

        // Static friction fallback when target velocity is zero.
        if (Math.abs(targetVelocity) < ZERO) {
            if (Math.abs(error) > deadband) {
                staticFriction = kS * Math.signum(error);
            } else {
                staticFriction = kS * Math.signum(effectiveKg);
            }
        }

        return staticFriction + (kV * targetVelocity) + (kA * targetAccel) + effectiveKg;
    }

    /**
     * Sets the strict position deadband. Motor output is set to 0 when error is within this bound.
     *
     * @param tol Position tolerance in units/ticks.
     */
    public void setPositionTolerance(double tol) {
        this.deadband = tol;
    }

    /**
     * Sets the settled/acceptance deadband. Used to determine if the state machine
     * is allowed to proceed while the motor continues refining position.
     *
     * @param tol Settled acceptance tolerance in units/ticks.
     */
    public void setSettledTolerance(double tol) {
        this.settledDeadband = tol;
    }

    /**
     * Sets upper and lower limits for the integral error sum (anti-windup safeguard).
     *
     * @param max Upper limit for integral accumulator.
     * @param min Lower limit for integral accumulator.
     */
    public void setIntegralLimits(double max, double min) {
        this.integralMin = Math.min(min, max);
        this.integralMax = Math.max(min, max);
    }

    /**
     * Enables or disables resetting the integral accumulator whenever error changes sign.
     *
     * @param reset True to clear integral sum on error sign change.
     */
    public void setResetIntegralOnSignChange(boolean reset) {
        this.resetIntegralOnSignChange = reset;
    }

    /**
     * Keeps kS/kV/kA/kG feedforward active after position error enters the deadband.
     * Useful for gravity-loaded arms/elevators that still need hold power at target.
     *
     * @param hold True to output feedforward in the deadband, false to output zero.
     */
    public void setHoldFeedforwardInDeadband(boolean hold) {
        this.holdFeedforwardInDeadband = hold;
    }

    /**
     * Checks if the position error is within the strict position deadband.
     *
     * @return True if absolute error is less than the position deadband.
     */
    public boolean isOnTarget() {
        return Math.abs(lastError) < deadband;
    }

    /**
     * Checks if the current error is within the user-defined settled deadband,
     * indicating that the mechanism state is acceptable to move on to the next action.
     *
     * @return True if absolute error is within the settled acceptance threshold.
     */
    public boolean isSettled() {
        return Math.abs(lastError) < settledDeadband;
    }

    /**
     * Bulk updates all PID and Feedforward coefficients.
     *
     * @param p Proportional gain (kP)
     * @param i Integral gain (kI)
     * @param d Derivative gain (kD)
     * @param s Static friction feedforward gain (kS)
     * @param v Velocity feedforward gain (kV)
     * @param a Acceleration feedforward gain (kA)
     * @param g Gravity feedforward gain (kG)
     */
    public void setPIDF(double p, double i, double d, double s, double v, double a, double g) {
        this.kP = p; this.kI = i; this.kD = d;
        this.kS = s; this.kV = v; this.kA = a; this.kG = g;
    }

    /**
     * Bulk updates PID and motion Feedforward coefficients (sets kG to 0).
     */
    public void setPIDF(double p, double i, double d, double s, double v, double a) {
        setPIDF(p, i, d, s, v, a, 0.0);
    }

    /** @param kP Proportional gain. */
    public void setkP(double kP) { this.kP = kP; }

    /** @param kI Integral gain. */
    public void setkI(double kI) { this.kI = kI; }

    /** @param kD Derivative gain. */
    public void setkD(double kD) { this.kD = kD; }

    /** @param kS Static friction gain. */
    public void setkS(double kS) { this.kS = kS; }

    /** @param kV Velocity feedforward gain. */
    public void setkV(double kV) { this.kV = kV; }

    /** @param kA Acceleration feedforward gain. */
    public void setkA(double kA) { this.kA = kA; }

    /** @param kG Gravity feedforward gain. */
    public void setkG(double kG) { this.kG = kG; }

    /** @return Proportional gain (kP). */
    public double getkP() { return kP; }

    /** @return Integral gain (kI). */
    public double getkI() { return kI; }

    /** @return Derivative gain (kD). */
    public double getkD() { return kD; }

    /** @return Static friction gain (kS). */
    public double getkS() { return kS; }

    /** @return Velocity feedforward gain (kV). */
    public double getkV() { return kV; }

    /** @return Acceleration feedforward gain (kA). */
    public double getkA() { return kA; }

    /** @return Gravity feedforward gain (kG). */
    public double getkG() { return kG; }

    /** @return Current integral error sum. */
    public double getErrorSum() { return errorSum; }

    /** @return Last measured position value. */
    public double getLastMeasurement() { return lastMeasurement; }

    /** @return Last calculated error value. */
    public double getLastError() { return lastError; }

    /** @return Last calculated motor output power. */
    public double getOutput() { return output; }

    /** @return Delta time of previous cycle in seconds. */
    public double getLastDt() { return lastDt; }

    /** @return Measured delta time of previous cycle in seconds. */
    public double getLastMeasuredDt() { return lastMeasuredDt; }

    /** @return Last change in integral sum. */
    public double getLastIntegralDelta() { return lastIntegralDelta; }

    /** @return Total number of times reset() was called. */
    public int getResetCount() { return resetCount; }

    /** @return True if integral sum resets on sign change. */
    public boolean getResetIntegralOnSignChange() { return resetIntegralOnSignChange; }

    /** @return True if feedforward remains active inside the position deadband. */
    public boolean getHoldFeedforwardInDeadband() { return holdFeedforwardInDeadband; }

    /** @return Upper limit of integral error sum. */
    public double getIntegralMax() { return integralMax; }

    /** @return Lower limit of integral error sum. */
    public double getIntegralMin() { return integralMin; }

    /** @return Strict position tolerance deadband. */
    public double getDeadband() { return deadband; }

    /** @return Settled acceptance deadband. */
    public double getSettledDeadband() { return settledDeadband; }

    /**
     * Resets the internal state of the controller, clearing integral sum,
     * velocity tracking, and error histories.
     */
    public void reset() {
        errorSum = 0;
        lastError = 0;
        lastMeasurement = 0;
        lastTargetVelocity = 0;
        output = 0;
        lastDt = 0;
        lastMeasuredDt = 0;
        lastIntegralDelta = 0;
        resetCount++;
        isFirstRun = true;
        timer.reset();
    }
}
