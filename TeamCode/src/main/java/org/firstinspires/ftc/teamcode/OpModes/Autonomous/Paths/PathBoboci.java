package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class PathBoboci {
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;

    public PathBoboci(Follower follower) {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(71.925, 135.626),

                                new Pose(27.514, 73.234)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(27.514, 73.234),

                                new Pose(70.206, 30.729)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(70.206, 30.729),

                                new Pose(129.897, 54.364)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(129.897, 54.364),

                                new Pose(81.794, 78.280)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(81.794, 78.280),

                                new Pose(12.654, 11.916)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }
}

