package org.firstinspires.ftc.teamcode.PanelsParameters;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

@Configurable
public class Arm2DOFTelemetry {
    @Sorter(sort = 1)
    public static double timeSeconds = 0.0;
    @Sorter(sort = 2)
    public static double joint1TargetDegrees = 0.0;
    @Sorter(sort = 3)
    public static double joint1PositionDegrees = 0.0;
    @Sorter(sort = 4)
    public static double joint1ErrorDegrees = 0.0;
    @Sorter(sort = 5)
    public static double joint1VelocityDegreesPerSecond = 0.0;
    @Sorter(sort = 6)
    public static double joint1Power = 0.0;
    @Sorter(sort = 7)
    public static double joint1PidOutput = 0.0;
    @Sorter(sort = 8)
    public static double joint1DynamicKg = 0.0;
    @Sorter(sort = 9)
    public static double joint1CurrentAmps = 0.0;

    @Sorter(sort = 10)
    public static double joint2TargetDegrees = 0.0;
    @Sorter(sort = 11)
    public static double joint2PositionDegrees = 0.0;
    @Sorter(sort = 12)
    public static double joint2ErrorDegrees = 0.0;
    @Sorter(sort = 13)
    public static double joint2VelocityDegreesPerSecond = 0.0;
    @Sorter(sort = 14)
    public static double joint2Power = 0.0;
    @Sorter(sort = 15)
    public static double joint2PidOutput = 0.0;
    @Sorter(sort = 16)
    public static double joint2DynamicKg = 0.0;
    @Sorter(sort = 17)
    public static double joint2CurrentAmps = 0.0;

    @Sorter(sort = 18)
    public static double shoulderMathDegrees = 0.0;
    @Sorter(sort = 19)
    public static double elbowMathDegrees = 0.0;
    @Sorter(sort = 20)
    public static double forearmAbsoluteDegrees = 0.0;
    @Sorter(sort = 21)
    public static double shoulderGravityRatio = 0.0;
    @Sorter(sort = 22)
    public static double elbowGravityRatio = 0.0;

    @Sorter(sort = 23)
    public static boolean joint1TargetLimited = false;
    @Sorter(sort = 24)
    public static boolean joint2TargetLimited = false;
    @Sorter(sort = 25)
    public static boolean joint1Settled = false;
    @Sorter(sort = 26)
    public static boolean joint2Settled = false;
    @Sorter(sort = 27)
    public static boolean joint1OverCurrent = false;
    @Sorter(sort = 28)
    public static boolean joint2OverCurrent = false;
    @Sorter(sort = 29)
    public static boolean profiling = false;
}
