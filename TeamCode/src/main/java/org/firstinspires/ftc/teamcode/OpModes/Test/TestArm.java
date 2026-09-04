package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Config.RobotHardware;
import org.firstinspires.ftc.teamcode.Modules.ClosedLoopDC;
import org.firstinspires.ftc.teamcode.Modules.PIDController;
import org.firstinspires.ftc.teamcode.Modules.SafeDcMotor;

@TeleOp(name="Testicle", group = "ZZZZZ")
public class TestArm extends OpMode {
    RobotHardware robotHardware;
    int j1target = 0, j2target = 0 ;
    int selMotor = 0;
    @Override
    public void init() {
        robotHardware = new RobotHardware(hardwareMap);
        robotHardware.cldcjoint1.init(true);
        robotHardware.cldcjoint2.init(true);
        robotHardware.cldcjoint1.disableAngleLimits();
        robotHardware.cldcjoint2.disableAngleLimits();
        robotHardware.cldcjoint1.setSCurveConstraints(112,3000,10000);
        robotHardware.cldcjoint2.setSCurveConstraints(112,3000,10000);
        robotHardware.cldcjoint1.enable(true);
        robotHardware.cldcjoint2.enable(true);
        robotHardware.cldcjoint2.setAngleMode(true);
        robotHardware.cldcjoint1.setAngleMode(true);
    }

    @Override
    public void loop() {
        robotHardware.cldcjoint2.update(j2target*0.1);
        robotHardware.cldcjoint1.update(j1target*0.1);
        if(gamepad1.dpad_right) selMotor = 1;
        if(gamepad1.dpad_left) selMotor = 0;
        if(selMotor == 0){
            if(gamepad1.dpad_up) j1target++;
            if(gamepad1.dpad_down) j1target--;
        } else if(selMotor == 1){
            if(gamepad1.dpad_up) j2target++;
            if(gamepad1.dpad_down) j2target--;
        }
        telemetry.addData("Sel Joint", selMotor + 1);
        telemetry.addData("J1 target",j1target);
        telemetry.addData("J2 target",j2target);
        robotHardware.cldcjoint1.appendTelemetry(telemetry,"J1");
        robotHardware.cldcjoint2.appendTelemetry(telemetry,"J2");
        telemetry.update();

    }
}
