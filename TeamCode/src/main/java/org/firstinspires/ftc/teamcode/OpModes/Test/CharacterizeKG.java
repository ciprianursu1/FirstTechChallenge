package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.PanelsParameters.SysIdParameters;

@TeleOp(name = "SysId: kG Characterization", group = "Tuning")
public class CharacterizeKG extends LinearOpMode {
    private DcMotorEx motor;
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime dtTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, SysIdParameters.motorName);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("=== kG HOLD TEST ===");
        telemetry.addLine("Set known kS and signs in SysIdParameters first.");
        telemetry.addLine("Place mechanism at the test position before pressing PLAY.");
        telemetry.addLine("Press CIRCLE to emergency stop.");
        telemetry.update();

        waitForStart();

        double holdPowerMagnitude = 0.0;
        double runningKgSum = 0.0;
        int validSamples = 0;
        boolean emergencyStopped = false;

        runtime.reset();
        dtTimer.reset();

        while (opModeIsActive()
                && runtime.seconds() < SysIdParameters.kgTestDuration) {
            if (gamepad1.circle || gamepad2.circle) {
                emergencyStopped = true;
                break;
            }

            double dt = dtTimer.seconds();
            if (dt < 1e-4) continue;
            dtTimer.reset();

            double signedVelocity =
                    motor.getVelocity() * SysIdParameters.encoderVelocitySign;

            holdPowerMagnitude +=
                    -signedVelocity * SysIdParameters.holdAdjustRate * dt;
            holdPowerMagnitude =
                    Math.max(
                            0.0,
                            Math.min(SysIdParameters.maxHoldPower, holdPowerMagnitude));

            double commandedPower =
                    holdPowerMagnitude * SysIdParameters.gravityPowerSign;
            motor.setPower(commandedPower);

            double gravityPower =
                    commandedPower
                            - (SysIdParameters.knownKS * Math.signum(commandedPower));
            double estimatedKg = gravityPower;
            boolean validSample =
                    Math.abs(signedVelocity) < SysIdParameters.holdVelocityDeadband
                            && Math.abs(commandedPower) > SysIdParameters.knownKS;

            if (SysIdParameters.armMode) {
                double cosine =
                        Math.cos(
                                SysIdParameters.getAngleRadians(
                                        motor.getCurrentPosition()));
                validSample =
                        validSample
                                && Math.abs(cosine)
                                        > SysIdParameters.minCosineMagnitude;
                estimatedKg = gravityPower / cosine;
            }

            if (validSample) {
                runningKgSum += estimatedKg;
                validSamples++;
            }

            telemetry.addData("Hold Power", "%.4f", commandedPower);
            telemetry.addData("Velocity", "%.1f ticks/s", signedVelocity);
            telemetry.addData("Instant kG", "%.5f", estimatedKg);
            telemetry.addData("Samples", validSamples);
            telemetry.update();
        }

        motor.setPower(0);

        double finalKg = validSamples > 0 ? runningKgSum / validSamples : 0.0;

        telemetry.clearAll();
        if (emergencyStopped) {
            telemetry.addLine("=== EMERGENCY STOP TRIGGERED ===");
        } else {
            telemetry.addLine("=== TEST COMPLETE ===");
            telemetry.addData(">> CALCULATED kG <<", "%.5f", finalKg);
            telemetry.addData("Samples", validSamples);
        }
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }
}
