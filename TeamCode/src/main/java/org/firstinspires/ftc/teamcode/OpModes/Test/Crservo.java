package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Crservo extends OpMode {
    Servo servo;
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class,"servo1");
    }

    @Override
    public void loop() {
        servo.setPosition(-gamepad1.left_stick_y);
        telemetry.addData("Power",-gamepad1.left_stick_y);
        telemetry.update();
    }
}
