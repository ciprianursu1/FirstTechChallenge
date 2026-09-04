package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Motor Test", group = "ZZZZ")
public class MotorTest extends OpMode {
    DcMotorEx joint1;
    DcMotorEx joint2;
    int motor = 1;
    @Override
    public void init() {
        joint1 = hardwareMap.get(DcMotorEx.class,"joint1");
        joint2 = hardwareMap.get(DcMotorEx.class,"joint2");
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_left){
            motor++;
        } else if(gamepad1.dpad_right){
            motor--;
        }
        if(motor%2 == 0) {
            telemetry.addLine("Motor 1");
            telemetry.addData("Pos", joint1.getCurrentPosition());
            joint1.setPower(-gamepad1.left_stick_y);
            joint2.setPower(0);
        }
        else if(motor%2 == 1){
            telemetry.addLine("Motor 2");
            telemetry.addData("Pos", joint2.getCurrentPosition());
            joint2.setPower(-gamepad1.left_stick_x);
            joint1.setPower(0);
        }
        telemetry.update();
    }
}
