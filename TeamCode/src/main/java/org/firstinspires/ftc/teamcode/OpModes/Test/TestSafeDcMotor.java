package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Modules.SafeDcMotor;
import org.firstinspires.ftc.teamcode.PanelsParameters.ModuleTestParameters;

@TeleOp(name = "Test: SafeDcMotor", group = "Module Tests")
public class TestSafeDcMotor extends OpMode {
    private SafeDcMotor motor;

    @Override
    public void init() {
        motor = new SafeDcMotor(hardwareMap, ModuleTestParameters.motorName);

        if (ModuleTestParameters.enableSlewRate) {
            motor.enableSlewRateLimiting(ModuleTestParameters.maxSlewRate);
        }
        if (ModuleTestParameters.enableVoltageCompensation) {
            motor.enableVoltageCompensation(ModuleTestParameters.nominalVoltage);
        }
        if (ModuleTestParameters.enableStallProtection) {
            motor.enableStallProtection(
                    ModuleTestParameters.stallCurrentAmps,
                    ModuleTestParameters.stallTimeoutMs);
        }

        if (ModuleTestParameters.resetEncoderOnInit) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        double requestedPower = -gamepad1.left_stick_y * ModuleTestParameters.manualPowerScale;
        if (gamepad1.a || gamepad1.cross) {
            requestedPower = 0.0;
        }
        if (gamepad1.x || gamepad1.square) {
            motor.resetStallTrip();
        }

        motor.setPower(requestedPower);
        motor.update();

        telemetry.addLine("SafeDcMotor Manual Test");
        telemetry.addData("Requested Power", "%.3f", requestedPower);
        telemetry.addData("Applied Power", "%.3f", motor.getPower());
        telemetry.addData("Position", motor.getCurrentPosition());
        telemetry.addData("Velocity", "%.1f", motor.getVelocity());
        telemetry.addData("Current", "%.2f A", motor.getCurrentAmps());
        telemetry.addData("Stalled", motor.isStalled());
        telemetry.addLine("Left stick Y drives. A/CROSS stops. X/SQUARE clears stall trip.");
    }

    @Override
    public void stop() {
        if (motor != null) {
            motor.setPower(0);
            motor.update();
        }
    }
}
