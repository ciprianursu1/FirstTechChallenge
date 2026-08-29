package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "ExampleTeleOp", group = "ZZZZZZZ")
public class ExampleTeleOp extends OpMode {
    private Follower follower;
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(0,0,0));
    }
    @Override
    public void start() {
        follower.startTeleopDrive();
    }
    @Override
    public void loop() {
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );
        telemetry.addData("Pose",follower.getPose().toString());
        follower.update();
    }
}
