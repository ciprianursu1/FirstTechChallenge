package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public  class PathORIG {
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;
    public PathChain Path10;
    public PathChain Path11;
    public PathChain Path12;
    public PathChain Path13;

    public PathORIG(Follower follower) {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(10.000, 72.000),

                                new Pose(60.000, 48.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(270.0))
                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.000, 48.000),

                                new Pose(60.000, 11.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270.0), Math.toRadians(270.0))
                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(60.000, 11.000),

                                new Pose(28.000, 198.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270.0), Math.toRadians(180.0))
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(28.000, 198.000),

                                new Pose(16.000, 198.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.000, 198.000),

                                new Pose(36.000, 48.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(270.0))
                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(36.000, 48.000),

                                new Pose(36.000, 11.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270.0), Math.toRadians(270.0))
                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(36.000, 11.000),

                                new Pose(28.000, 198.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270.0), Math.toRadians(180.0))
                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(28.000, 198.000),

                                new Pose(14.000, 198.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(14.000, 198.000),

                                new Pose(12.000, 48.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(270.0))
                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.000, 48.000),

                                new Pose(12.000, 12.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270.0), Math.toRadians(270.0))
                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.000, 12.000),

                                new Pose(28.000, 198.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270.0), Math.toRadians(180.0))
                .build();

        Path12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(28.000, 198.000),

                                new Pose(16.000, 198.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
                .build();

        Path13 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.000, 198.000),

                                new Pose(58.000, 61.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(0.0))
                .build();

    }
}