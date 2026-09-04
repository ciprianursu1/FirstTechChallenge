package org.firstinspires.ftc.teamcode.PanelsParameters;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

@Configurable
public class ModuleTestParameters {
    @Sorter(sort = 1)
    public static String motorName = "armMotor";
    @Sorter(sort = 2)
    public static String limelightName = "limelight";

    @Sorter(sort = 3)
    public static boolean resetEncoderOnInit = false;
    @Sorter(sort = 4)
    public static double manualPowerScale = 0.35;
    @Sorter(sort = 5)
    public static double maxPower = 0.45;
    @Sorter(sort = 6)
    public static double targetPosition = 0.0;
    @Sorter(sort = 7)
    public static double targetStep = 50.0;
    @Sorter(sort = 8)
    public static boolean useProfile = true;
    @Sorter(sort = 9)
    public static boolean angleMode = false;
    @Sorter(sort = 10)
    public static boolean angleWrapping = true;
    @Sorter(sort = 11)
    public static boolean enableAngleLimits = false;
    @Sorter(sort = 12)
    public static boolean useLargeArcAngleLimit = false;
    @Sorter(sort = 13)
    public static double firstAngleLimitDegrees = -90.0;
    @Sorter(sort = 14)
    public static double secondAngleLimitDegrees = 90.0;
    @Sorter(sort = 15)
    public static double ticksPerRev = 384.5;

    @Sorter(sort = 16)
    public static boolean enableCosineGravity = false;
    @Sorter(sort = 17)
    public static double gravityHorizontalTicks = 0.0;
    @Sorter(sort = 18)
    public static double gravityPowerSign = 1.0;
    @Sorter(sort = 19)
    public static boolean holdFeedforwardInDeadband = true;

    @Sorter(sort = 20)
    public static double maxVel = 1200.0;
    @Sorter(sort = 21)
    public static double maxAccel = 2400.0;
    @Sorter(sort = 22)
    public static double maxJerk = 12000.0;
    @Sorter(sort = 23)
    public static double profileDistance = 1000.0;

    @Sorter(sort = 24)
    public static boolean enableSlewRate = true;
    @Sorter(sort = 25)
    public static double maxSlewRate = 2.0;
    @Sorter(sort = 26)
    public static boolean enableVoltageCompensation = false;
    @Sorter(sort = 27)
    public static double nominalVoltage = 12.0;
    @Sorter(sort = 28)
    public static boolean enableStallProtection = false;
    @Sorter(sort = 29)
    public static double stallCurrentAmps = 6.5;
    @Sorter(sort = 30)
    public static double stallTimeoutMs = 300.0;

    @Sorter(sort = 31)
    public static int limelightPipeline = 0;
    @Sorter(sort = 32)
    public static int limelightTargetTagId = -1;
    @Sorter(sort = 33)
    public static boolean useMegaTag2 = false;
    @Sorter(sort = 34)
    public static double robotHeadingDegrees = 0.0;

    @Sorter(sort = 35)
    public static int ringBufferSize = 100;
    @Sorter(sort = 36)
    public static double ringBufferQueryLatencyMs = 120.0;
}
