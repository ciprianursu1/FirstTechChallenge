package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.PanelsParameters.SysIdParameters;

@TeleOp(name = "SysId: kS & kV Characterization", group = "Tuning")
public class CharacterizeFeedforward extends LinearOpMode {
    private DcMotorEx motor;
    private final ElapsedTime stepTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, SysIdParameters.motorName);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("=== FEEDFORWARD CHARACTERIZATION ===");
        telemetry.addLine("Tune kG first for arms/elevators, then run this test.");
        telemetry.addLine("Press CIRCLE to emergency stop.");
        telemetry.update();

        waitForStart();
        stepTimer.reset();

        double rampPower = 0.0;
        double detectedKS = -1.0;
        double runningKvSum = 0.0;
        int validKvSamples = 0;
        boolean kSFound = false;
        boolean emergencyStopped = false;

        while (opModeIsActive() && rampPower <= SysIdParameters.maxRampPower) {
            if (gamepad1.circleWasPressed() || gamepad2.circleWasPressed()) {
                emergencyStopped = true;
                break;
            }

            double dt = stepTimer.seconds();
            stepTimer.reset();

            rampPower += SysIdParameters.rampRatePerSec * dt;
            rampPower = Math.min(rampPower, SysIdParameters.maxRampPower);

            double gravityPower =
                    SysIdParameters.getGravityFeedforward(motor.getCurrentPosition());
            double motorPower = SysIdParameters.clampMotorPower(
                    gravityPower
                            + (SysIdParameters.characterizationPowerSign * rampPower));
            motor.setPower(motorPower);

            double movementPower =
                    Math.max(
                            0.0,
                            (motorPower - gravityPower)
                                    * SysIdParameters.characterizationPowerSign);
            double signedVelocity = motor.getVelocity() * SysIdParameters.encoderVelocitySign;
            double absVelocity = Math.abs(signedVelocity);

            if (!kSFound && absVelocity > SysIdParameters.startThresholdVelocity) {
                detectedKS = movementPower;
                kSFound = true;
            }

            double instantKV = 0.0;
            if (kSFound
                    && absVelocity > SysIdParameters.startThresholdVelocity
                    && movementPower > detectedKS) {
                instantKV = (movementPower - detectedKS) / absVelocity;
                runningKvSum += instantKV;
                validKvSamples++;
            }

            telemetry.addData("Status", "Ramping Power...");
            telemetry.addData("Ramp Power", "%.3f", rampPower);
            telemetry.addData("Movement Power", "%.3f", movementPower);
            telemetry.addData("Gravity Power", "%.3f", gravityPower);
            telemetry.addData("Commanded Power", "%.3f", motorPower);
            telemetry.addData("Velocity", "%.1f ticks/s", signedVelocity);
            telemetry.addData("Instant kV", "%.7f", instantKV);

            if (kSFound) {
                telemetry.addData(">> ESTIMATED kS <<", "%.4f", detectedKS);
            } else {
                telemetry.addData(">> ESTIMATED kS <<", "Searching...");
            }

            telemetry.update();
            sleep(20);
        }

        motor.setPower(0);

        double estimatedKV = validKvSamples > 0 ? runningKvSum / validKvSamples : 0.0;

        telemetry.clearAll();
        if (emergencyStopped) {
            telemetry.addLine("=== EMERGENCY STOP TRIGGERED ===");
        } else {
            telemetry.addLine("=== TEST COMPLETE ===");
            telemetry.addData("Final Estimated kS", "%.4f", detectedKS);
            telemetry.addData("Final Estimated kV", "%.7f (Power per tick/sec)", estimatedKV);
            telemetry.addData("kV Samples", validKvSamples);
        }
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }
}
