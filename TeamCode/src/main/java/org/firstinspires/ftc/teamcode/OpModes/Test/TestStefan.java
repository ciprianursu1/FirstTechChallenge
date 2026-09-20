package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Stefan stie soft")
public class TestStefan extends OpMode {
    DcMotorEx joint1;
    DcMotorEx joint2;
    double target1 = 0, target2 = 0;
    final double tick_per_deg = 5240/360.0;
    @Override
    public void init() {
        joint1 = hardwareMap.get(DcMotorEx.class,"joint1");
        joint2 = hardwareMap.get(DcMotorEx.class,"joint2");
        joint1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        joint2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        joint1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        joint2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        joint1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        joint2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_left) target1-=0.1;
        if(gamepad1.dpad_right) target1+=0.1;
        if(gamepad1.dpad_down) target2-=0.1;
        if(gamepad1.dpad_up) target2+=0.1;
        int targetTicks1 = (int) (target1 * tick_per_deg);
        int targetTicks2 = (int) (target2 * tick_per_deg);
        joint1.setTargetPosition(targetTicks1);
        joint1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        joint1.setPower(0.5);
        joint2.setTargetPosition(targetTicks2);
        joint2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        joint2.setPower(0.5);

        telemetry.addData("Joint 1 target deg", target1);
        telemetry.addData("Joint 1 target ticks", targetTicks1);
        telemetry.addData("Joint 1 current ticks", joint1.getCurrentPosition());
        telemetry.addData("Joint 2 target deg", target2);
        telemetry.addData("Joint 2 target ticks", targetTicks2);
        telemetry.addData("Joint 2 current ticks", joint2.getCurrentPosition());
        telemetry.update();
    }
}
