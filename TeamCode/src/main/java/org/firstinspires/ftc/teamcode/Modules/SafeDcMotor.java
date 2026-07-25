package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Wrapper class around {@link DcMotorEx} providing integrated electrical safety,
 * motor thermal protection, and power compensation features.
 * <p>
 * Key Features:
 * <ul>
 * <li><b>Slew Rate Limiting:</b> Smooths out $0 \rightarrow 100\%$ instantaneous power demands to eliminate battery voltage sags.</li>
 * <li><b>Voltage Feedforward Compensation:</b> Dynamically scales target motor power based on live battery voltage to maintain consistent performance as the battery drains.</li>
 * <li><b>Current Monitoring & Stall Protection:</b> Continuously tracks motor amperage draw to detect mechanical jams and automatically cuts power before damage or blown fuses occur.</li>
 * </ul>
 * </p>
 *
 * @author FTC Team Code
 * @version 1.0
 */
public class SafeDcMotor {

    private final DcMotorEx motor;
    private final VoltageSensor voltageSensor;

    // --- Slew Rate Limiting ---
    private boolean slewRateEnabled = false;
    private double maxSlewRate = 2.0; // Units per second (e.g., 2.0 = 0 to 1.0 power in 0.5s)
    private double lastAppliedPower = 0.0;
    private long lastSlewTimeNs = System.nanoTime();

    // --- Voltage Compensation ---
    private boolean voltageCompensationEnabled = false;
    private double nominalVoltage = 12.0; // Standard nominal voltage target

    // --- Current / Stall Protection ---
    private boolean stallProtectionEnabled = false;
    private double stallCurrentThresholdAmps = 6.5; // Default threshold in Amperes
    private long stallTimeoutNs = 300_000_000L; // 300 ms in nanoseconds
    private long stallStartTimeNs = 0;
    private boolean isStalled = false;

    // --- Target Power State ---
    private double targetPower = 0.0;

    /**
     * Constructs a SafeDcMotor wrapper around a hardware {@link DcMotorEx}.
     *
     * @param hardwareMap Active hardware map reference.
     * @param deviceName  Configuration name of the motor in HardwareMap.
     */
    public SafeDcMotor(HardwareMap hardwareMap, String deviceName) {
        this.motor = hardwareMap.get(DcMotorEx.class, deviceName);

        // Attempt to retrieve primary VoltageSensor
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    /**
     * Constructs a wrapper directly from an existing {@link DcMotorEx} instance.
     *
     * @param motor         Configured {@link DcMotorEx} instance.
     * @param voltageSensor Voltage sensor reference for battery scaling.
     */
    public SafeDcMotor(DcMotorEx motor, VoltageSensor voltageSensor) {
        this.motor = motor;
        this.voltageSensor = voltageSensor;
    }

    // ==========================================
    // CONFIGURATION BUILDER METHODS
    // ==========================================

    /**
     * Enables Slew Rate Limiting on motor power updates.
     *
     * @param maxRateOfChange Maximum power change allowed per second (e.g., 2.0 = 0% to 100% in 0.5s).
     * @return This instance for method chaining.
     */
    public SafeDcMotor enableSlewRateLimiting(double maxRateOfChange) {
        this.slewRateEnabled = true;
        this.maxSlewRate = maxRateOfChange;
        this.lastSlewTimeNs = System.nanoTime();
        return this;
    }

    /**
     * Enables Voltage Compensation scaling. Motor output power will automatically scale upwards
     * when battery voltage drops below nominal level (and vice versa).
     *
     * @param nominalVoltage Baseline voltage (typically 12.0V).
     * @return This instance for method chaining.
     */
    public SafeDcMotor enableVoltageCompensation(double nominalVoltage) {
        this.voltageCompensationEnabled = true;
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    /**
     * Enables Current Monitoring to automatically shut down motor output if current stays
     * above a threshold for longer than the specified duration.
     *
     * @param maxAmps         Stall threshold in Amperes (e.g., 6.5A).
     * @param durationMs Duration in milliseconds the overload can persist before tripping (e.g., 300ms).
     * @return This instance for method chaining.
     */
    public SafeDcMotor enableStallProtection(double maxAmps, double durationMs) {
        this.stallProtectionEnabled = true;
        this.stallCurrentThresholdAmps = maxAmps;
        this.stallTimeoutNs = (long) (durationMs * 1e6);
        return this;
    }

    // ==========================================
    // PERIODIC UPDATE LOGIC
    // ==========================================

    /**
     * Sets the requested motor power. Safe transformations (Slew Limiting, Voltage Scaling,
     * and Stall Safeguards) will be computed during {@link #update()}.
     *
     * @param power Requested power in range [-1.0, 1.0].
     */
    public void setPower(double power) {
        this.targetPower = Math.max(-1.0, Math.min(1.0, power));
    }

    /**
     * Must be called periodically inside your OpMode control loop.
     * Evaluates stall current status, applies slew limiting, scales power for voltage,
     * and writes final power to hardware.
     */
    public void update() {
        long currentTimeNs = System.nanoTime();

        // --- 1. Check Stall / Current Guard ---
        if (stallProtectionEnabled) {
            double currentAmps = motor.getCurrent(CurrentUnit.AMPS);

            if (currentAmps > stallCurrentThresholdAmps) {
                if (stallStartTimeNs == 0) {
                    stallStartTimeNs = currentTimeNs; // Start stall timer
                } else if ((currentTimeNs - stallStartTimeNs) > stallTimeoutNs) {
                    isStalled = true; // Tripped stall protection!
                }
            } else {
                stallStartTimeNs = 0; // Reset timer when current drops back to safe level
            }
        }

        // If trip conditions are met, cut power immediately for safety!
        if (isStalled) {
            motor.setPower(0.0);
            return;
        }

        double commandedPower = targetPower;

        // --- 2. Calculate Slew Rate Limiting ---
        if (slewRateEnabled) {
            double dt = (currentTimeNs - lastSlewTimeNs) / 1e9; // Convert ns to seconds
            lastSlewTimeNs = currentTimeNs;

            double maxAllowedDelta = maxSlewRate * dt;
            double delta = commandedPower - lastAppliedPower;

            // Clamp rate of change
            double clampedDelta = Math.max(-maxAllowedDelta, Math.min(maxAllowedDelta, delta));
            commandedPower = lastAppliedPower + clampedDelta;
            lastAppliedPower = commandedPower;
        }

        // --- 3. Calculate Voltage Feedforward Compensation ---
        if (voltageCompensationEnabled && voltageSensor != null) {
            double currentVoltage = voltageSensor.getVoltage();
            if (currentVoltage > 6.0) { // Avoid divide-by-zero or brownout garbage readings
                double scaleFactor = nominalVoltage / currentVoltage;
                commandedPower *= scaleFactor;
            }
        }

        // --- 4. Final Power Delivery ---
        commandedPower = Math.max(-1.0, Math.min(1.0, commandedPower));
        motor.setPower(commandedPower);
    }

    /**
     * Resets a tripped stall protection state back to normal operating mode.
     */
    public void resetStallTrip() {
        this.isStalled = false;
        this.stallStartTimeNs = 0;
    }

    /**
     * Returns true if the motor current overload protection is currently tripped.
     */
    public boolean isStalled() {
        return isStalled;
    }

    /**
     * Reads live motor electrical draw in Amperes.
     */
    public double getCurrentAmps() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }

    // ==========================================
    // HARDWARE DELEGATION METHODS
    // ==========================================

    public void setMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setTargetPosition(int position) {
        motor.setTargetPosition(position);
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public double getPower() {
        return motor.getPower();
    }

    public DcMotorEx getInternalMotor() {
        return motor;
    }
    public double getVelocity() {
        return motor.getVelocity();
    }
    public double getCurrent(CurrentUnit currentUnit) {
        return motor.getCurrent(currentUnit);
    }
    public boolean isOverCurrent() {
        return motor.isOverCurrent();
    }
}