package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name = "SysId: kS & kV Characterization", group = "Tuning")
public class CharacterizeFeedforward extends LinearOpMode {

    // --- Configuration Constants ---
    private static final String MOTOR_NAME = "armMotor"; // Change to your motor name
    private static final double RAMP_RATE_PER_SEC = 0.05;  // Power increase per second (0.05 = 20s to max)
    private static final double START_THRESHOLD_VELOCITY = 10.0; // Ticks/sec to register as "moving"

    private DcMotorEx motor;
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime stepTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("=== FEEDFORWARD CHARACTERIZATION ===");
        telemetry.addLine("1. Make sure mechanism is flat or has free range of motion!");
        telemetry.addLine("2. Press PLAY to start automated voltage ramp-up.");
        telemetry.addLine("3. Press CIRCLE to emergency stop!");
        telemetry.update();

        waitForStart();
        runtime.reset();
        stepTimer.reset();

        double power = 0.0;
        double detectedKS = -1;
        boolean kSFound = false;
        boolean emergencyStop = false;
        double maxVelocity = 0;


        while (opModeIsActive() && power <= 1.0) {
            if(gamepad1.circleWasPressed() || gamepad2.circleWasPressed()) emergencyStop = true;
            double dt = stepTimer.seconds();
            stepTimer.reset();

            // Gradually ramp up motor power over time
            power += RAMP_RATE_PER_SEC * dt;
            power = Math.min(power, 1.0); // Clamp to 1.0
            if(!emergencyStop) motor.setPower(power);
            else {
                motor.setPower(0);
                telemetry.clearAll();
                telemetry.addLine("EMERGENCY STOP");
                telemetry.update();
                requestOpModeStop();
            }
            double velocity = motor.getVelocity();
            maxVelocity = Math.max(Math.abs(velocity),maxVelocity);
            // Detect kS (stiction break threshold)
            if (!kSFound && Math.abs(velocity) > START_THRESHOLD_VELOCITY) {
                detectedKS = power;
                kSFound = true;
            }

            // Driver Station Display
            telemetry.addData("Status", "Ramping Power...");
            telemetry.addData("Commanded Power", "%.3f", power);
            telemetry.addData("Current Velocity", "%.1f ticks/s", velocity);

            if (kSFound) {
                telemetry.addData(">> ESTIMATED kS <<", "%.4f (Power)", detectedKS);
            } else {
                telemetry.addData(">> ESTIMATED kS <<", "Searching...");
            }

            telemetry.update();
            sleep(20); // ~50 Hz update loop
        }

        // Stop motor at end of test
        motor.setPower(0);

        // --- Calculate estimated kV from max speed ---
        double estimatedKV = (maxVelocity > 0) ? (1.0 / maxVelocity) : 0;

        telemetry.clearAll();
        telemetry.addLine("=== TEST COMPLETE ===");
        telemetry.addData("Final Estimated kS", "%.4f", detectedKS);
        telemetry.addData("Final Estimated kV", "%.6f (Power per tick/sec)", estimatedKV);
        telemetry.update();

        while (opModeIsActive()) {
            idle(); // Hold results on screen until stopped
        }
    }
}