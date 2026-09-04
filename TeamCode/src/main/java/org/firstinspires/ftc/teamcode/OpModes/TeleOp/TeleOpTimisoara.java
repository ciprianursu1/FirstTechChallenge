package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Config.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Timisoara", group = "AAAA")
public class TeleOpTimisoara extends OpMode {
    RobotHardware robotHardware;
    Follower follower;
    @Override
    public void init() {
        robotHardware = new RobotHardware(hardwareMap);
        robotHardware.cldcjoint2.disableAngleLimits();
        robotHardware.cldcjoint1.disableAngleLimits();
        robotHardware.cldcjoint1.setAngleMode(true);
        robotHardware.cldcjoint2.setAngleMode(true);
        robotHardware.cldcjoint1.enable(true);
        robotHardware.cldcjoint2.enable(true);
        robotHardware.cldcjoint1.init(true);
        robotHardware.cldcjoint2.init(true);
        follower = Constants.createFollower(hardwareMap);
        robotHardware.cldcjoint2.disableCosineGravityFeedforward();
        robotHardware.cldcjoint1.disableCosineGravityFeedforward();
    }

    @Override
    public void loop() {

    }
}
