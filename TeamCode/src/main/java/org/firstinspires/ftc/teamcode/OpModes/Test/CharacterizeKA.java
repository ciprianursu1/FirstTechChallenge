package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "SysId: kA Characterization", group = "Tuning")
public class CharacterizeKA extends LinearOpMode {

    // --- CONFIGURATION ---
    private static final String MOTOR_NAME = "armMotor";

    // Fill these in from your previous kS/kV test!
    private static final double KNOWN_KS = 0.08; // Static friction power offset
    private static final double KNOWN_KV = 0.0004; // Power per (ticks/sec)

    private static final double STEP_POWER = 0.70; // 70% step voltage
    private static final double TEST_DURATION = 1.5; // Seconds to run test

    private DcMotorEx motor;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime dtTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("=== kA DYNAMIC STEP TEST ===");
        telemetry.addLine("Enter your known kS and kV in the code constants first!");
        telemetry.addLine("Make sure mechanism has full room to accelerate rapidly.");
        telemetry.addLine("\n[!] Press CIRCLE on either gamepad at ANY time for Emergency Stop.");
        telemetry.addLine("\nPress PLAY to run step test.");
        telemetry.update();

        waitForStart();

        // High frequency loop variables
        double lastVelocity = 0;
        double filteredAccel = 0;
        double alpha = 0.3; // Low-pass filter coefficient for smooth acceleration

        double runningKaSum = 0;
        int validSamples = 0;
        boolean emergencyStopped = false;

        runtime.reset();
        dtTimer.reset();

        // Apply instant step power
        motor.setPower(STEP_POWER);

        while (opModeIsActive() && runtime.seconds() < TEST_DURATION) {
            // Check for Emergency Stop (Circle button on Gamepad 1 or Gamepad 2)
            if (gamepad1.circle || gamepad2.circle) {
                emergencyStopped = true;
                break;
            }

            double dt = dtTimer.seconds();
            if (dt < 1e-4) continue; // Safety guard against 0 division
            dtTimer.reset();

            double currentVelocity = motor.getVelocity();
            double rawAccel = (currentVelocity - lastVelocity) / dt;
            lastVelocity = currentVelocity;

            // Low-pass filter to clean up encoder jitter
            filteredAccel = (alpha * rawAccel) + ((1.0 - alpha) * filteredAccel);

            // Calculate instantaneous kA when mechanism is accelerating (a > 100 ticks/s^2)
            double calculatedKa = 0;
            if (filteredAccel > 100.0) {
                // Voltage left over strictly dedicated to inertia = Total Power - kS - (kV * v)
                double powerForInertia = STEP_POWER - KNOWN_KS - (KNOWN_KV * currentVelocity);
                calculatedKa = powerForInertia / filteredAccel;

                runningKaSum += calculatedKa;
                validSamples++;
            }

            telemetry.addData("Status", "Running Step Function...");
            telemetry.addData("Velocity", "%.1f ticks/s", currentVelocity);
            telemetry.addData("Filtered Accel", "%.1f ticks/s^2", filteredAccel);
            telemetry.addData("Instantaneous kA", "%.8f", calculatedKa);
            telemetry.addLine("\n[Press CIRCLE to Emergency Stop]");
            telemetry.update();
        }

        // Safety stop: cut motor power immediately
        motor.setPower(0);

        // Average the valid kA samples captured during ramp-up
        double finalKa = (validSamples > 0) ? (runningKaSum / validSamples) : 0;

        telemetry.clearAll();
        if (emergencyStopped) {
            telemetry.addLine("=== EMERGENCY STOP TRIGGERED ===");
            telemetry.addLine("Motor powered off safely.");
        } else {
            telemetry.addLine("=== TEST COMPLETE ===");
            telemetry.addData("Samples Captured", validSamples);
            telemetry.addData(">> CALCULATED kA <<", "%.8f (Power per tick/sec^2)", finalKa);
        }
        telemetry.update();

        while (opModeIsActive()) {
            idle(); // Keep final status on screen
        }
    }
}