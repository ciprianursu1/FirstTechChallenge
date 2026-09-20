package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import android.speech.RecognitionService;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "BratBoboci")
public class BratBoboci extends OpMode {
    DcMotorEx joint1;
    DcMotorEx joint2;
    Servo servo0;
    final double maxIncrement = 10;
    double j1target = 0;
    double j2target = 0;
    double j1last = 0;
    double j2last = 0;
    final double maxJ1 = 1700;
    final double minJ1 = -1300;
    boolean clawOpen = true;
    final double openPosition = 0;
    final double closedPosition = 0.23;

    @Override
    public void init() {
        joint1 = (DcMotorEx) hardwareMap.get(DcMotor.class,"joint2");
        joint2 = (DcMotorEx) hardwareMap.get(DcMotor.class, "joint1");
        servo0 = hardwareMap.get(Servo.class,"servo0");
        servo0.setDirection(Servo.Direction.REVERSE);
        joint1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        joint2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        joint1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        joint2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        joint1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        joint2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        joint1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10,0,0,0));
        joint2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10,0,0,0));
    }

    @Override
    public void loop() {
        j1target += gamepad2.left_stick_y * maxIncrement;
        j2target += gamepad2.right_stick_y * maxIncrement;
        if(gamepad1.rightBumperWasPressed()){
            j1target = 664;
            j2target = -1938;
        }
        if(gamepad2.rightBumperWasPressed()){
            clawOpen = !clawOpen;
        }
        if(clawOpen){
            servo0.setPosition(openPosition);
        } else {
            servo0.setPosition(closedPosition);
        }
        if(j1target > maxJ1) j1target = maxJ1;
        else if(j1target < minJ1) j1target = minJ1;
        joint1.setTargetPosition((int) j1target);
        joint1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        joint1.setPower(0.45);
        joint2.setTargetPosition((int) (j2target + j1target));
        joint2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        joint2.setPower(0.45);
        telemetry.addData("Pos J1", joint1.getCurrentPosition());
        telemetry.addData("Pos J2", joint2.getCurrentPosition());
        telemetry.addData("Target J1", j1target);
        telemetry.addData("Target J2", j2target);
        telemetry.update();
    }
}
