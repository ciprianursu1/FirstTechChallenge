package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "AutoBoboci")
public class AutoBoboci extends OpMode {
    Follower follower;
    int stage = 0;
    boolean pathStarted = false;
    PathBoboci paths;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(72,136,Math.toRadians(-90)));
        paths = new PathBoboci(follower);
    }

    @Override
    public void loop() {
        switch (stage){
            case 0:
                stage = 1;
                break;
            case 1:
                if(!pathStarted) {
                    follower.followPath(paths.Path1);
                    pathStarted = true;
                }
                if(!follower.isBusy() && pathStarted){
                    stage = 2;
                    pathStarted = false;
                }
                break;
            case 2:
                if(!pathStarted) {
                    follower.followPath(paths.Path2);
                    pathStarted = true;
                }
                if(!follower.isBusy() && pathStarted){
                    stage = 2;
                    pathStarted = false;
                }
                break;
            case 3:
                if(!pathStarted) {
                    follower.followPath(paths.Path3);
                    pathStarted = true;
                }
                if(!follower.isBusy() && pathStarted){
                    stage = 2;
                    pathStarted = false;
                }
                break;
            case 4:
                if(!pathStarted) {
                    follower.followPath(paths.Path4);
                    pathStarted = true;
                }
                if(!follower.isBusy() && pathStarted){
                    stage = 2;
                    pathStarted = false;
                }
                break;
            case 5:
                if(!pathStarted) {
                    follower.followPath(paths.Path5);
                    pathStarted = true;
                }
                if(!follower.isBusy() && pathStarted){
                    stage = 2;
                    pathStarted = false;
                    stop();
                }
                break;
        }
        follower.update();

    }
}
