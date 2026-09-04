package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Config.RobotHardware;

@TeleOp(name = "STEFAN E GAY")
public class SkibidiStefan extends OpMode {
    RobotHardware robotHardware;
    double j1target = 0;
    @Override
    public void init() {
        robotHardware = new RobotHardware(hardwareMap);
        robotHardware.cldcjoint1.init(true);
        robotHardware.cldcjoint2.init(true);
        robotHardware.cldcjoint1.enable(false);
        robotHardware.cldcjoint2.enable(false);

    }

    @Override
    public void loop() {
        telemetry.addData("Pos J2",robotHardware.cldcjoint2.getCurrentPosition());
        telemetry.addData("Pos J1",robotHardware.cldcjoint1.getCurrentPosition());
        telemetry.update();
    }
}
