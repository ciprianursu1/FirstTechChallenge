package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.PanelsParameters.SysIdParameters;

@TeleOp(name = "SysId: kA Characterization", group = "Tuning")
public class CharacterizeKA extends LinearOpMode {
    private DcMotorEx motor;
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime dtTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, SysIdParameters.motorName);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("=== kA DYNAMIC STEP TEST ===");
        telemetry.addLine("Enter known kS, kV, and kG in SysIdParameters first.");
        telemetry.addLine("Press CIRCLE on either gamepad for emergency stop.");
        telemetry.update();

        waitForStart();

        double lastVelocity = 0.0;
        double filteredAccel = 0.0;
        double runningKaSum = 0.0;
        int validSamples = 0;
        boolean emergencyStopped = false;

        runtime.reset();
        dtTimer.reset();

        while (opModeIsActive()
                && runtime.seconds() < SysIdParameters.stepTestDuration) {
            if (gamepad1.circle || gamepad2.circle) {
                emergencyStopped = true;
                break;
            }

            double dt = dtTimer.seconds();
            if (dt < 1e-4) continue;
            dtTimer.reset();

            double gravityPower =
                    SysIdParameters.getGravityFeedforward(motor.getCurrentPosition());
            double motorPower = SysIdParameters.clampMotorPower(
                    gravityPower
                            + (SysIdParameters.characterizationPowerSign
                                    * SysIdParameters.stepPower));
            motor.setPower(motorPower);

            double currentVelocity =
                    motor.getVelocity() * SysIdParameters.encoderVelocitySign;
            double rawAccel = (currentVelocity - lastVelocity) / dt;
            lastVelocity = currentVelocity;

            filteredAccel =
                    (SysIdParameters.accelerationFilterAlpha * rawAccel)
                            + ((1.0 - SysIdParameters.accelerationFilterAlpha)
                                    * filteredAccel);

            double signedMovementPower =
                    (motorPower - gravityPower)
                            * SysIdParameters.characterizationPowerSign;
            double calculatedKa = 0.0;

            if (filteredAccel > SysIdParameters.accelerationThreshold) {
                double powerForInertia =
                        signedMovementPower
                                - SysIdParameters.knownKS
                                - (SysIdParameters.knownKV * currentVelocity);
                calculatedKa = powerForInertia / filteredAccel;

                if (calculatedKa > 0) {
                    runningKaSum += calculatedKa;
                    validSamples++;
                }
            }

            telemetry.addData("Status", "Running Step Function...");
            telemetry.addData("Gravity Power", "%.3f", gravityPower);
            telemetry.addData("Commanded Power", "%.3f", motorPower);
            telemetry.addData("Velocity", "%.1f ticks/s", currentVelocity);
            telemetry.addData("Filtered Accel", "%.1f ticks/s^2", filteredAccel);
            telemetry.addData("Instantaneous kA", "%.8f", calculatedKa);
            telemetry.addLine("\n[Press CIRCLE to Emergency Stop]");
            telemetry.update();
        }

        motor.setPower(0);

        double finalKa = validSamples > 0 ? runningKaSum / validSamples : 0.0;

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
            idle();
        }
    }
}
