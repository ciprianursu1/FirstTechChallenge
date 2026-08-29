package org.firstinspires.ftc.teamcode.PanelsParameters;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

@Configurable
public class ClosedLoopDCTelemetry {
    @Sorter(sort = 1)
    public static double timeSeconds = 0.0;
    @Sorter(sort = 2)
    public static double target = 0.0;
    @Sorter(sort = 3)
    public static double position = 0.0;
    @Sorter(sort = 4)
    public static double error = 0.0;
    @Sorter(sort = 5)
    public static double velocity = 0.0;
    @Sorter(sort = 6)
    public static double commandedPower = 0.0;
    @Sorter(sort = 7)
    public static double pidOutput = 0.0;
    @Sorter(sort = 8)
    public static double effectiveKg = 0.0;
    @Sorter(sort = 9)
    public static double motorCurrentAmps = 0.0;

    @Sorter(sort = 10)
    public static double maxPower = 0.0;
    @Sorter(sort = 11)
    public static double pidTarget = 0.0;
    @Sorter(sort = 12)
    public static double pidCurrent = 0.0;
    @Sorter(sort = 13)
    public static double pidError = 0.0;
    @Sorter(sort = 14)
    public static double pidErrorSum = 0.0;
    @Sorter(sort = 15)
    public static double pidDt = 0.0;
    @Sorter(sort = 16)
    public static double pidIntegralDelta = 0.0;

    @Sorter(sort = 17)
    public static double kP = 0.0;
    @Sorter(sort = 18)
    public static double kI = 0.0;
    @Sorter(sort = 19)
    public static double kD = 0.0;
    @Sorter(sort = 20)
    public static double kS = 0.0;
    @Sorter(sort = 21)
    public static double kV = 0.0;
    @Sorter(sort = 22)
    public static double kA = 0.0;
    @Sorter(sort = 23)
    public static double kG = 0.0;
    @Sorter(sort = 24)
    public static double deadband = 0.0;
    @Sorter(sort = 25)
    public static double settledDeadband = 0.0;
    @Sorter(sort = 26)
    public static double integralMin = 0.0;
    @Sorter(sort = 27)
    public static double integralMax = 0.0;

    @Sorter(sort = 28)
    public static double profileTarget = 0.0;
    @Sorter(sort = 29)
    public static double profileTimeSeconds = 0.0;
    @Sorter(sort = 30)
    public static double profileTotalTimeSeconds = 0.0;
    @Sorter(sort = 31)
    public static double gravityAngleDegrees = 0.0;
    @Sorter(sort = 32)
    public static double horizontalTicks = 0.0;

    @Sorter(sort = 33)
    public static boolean enabled = false;
    @Sorter(sort = 34)
    public static boolean angleMode = false;
    @Sorter(sort = 35)
    public static boolean cosineGravity = false;
    @Sorter(sort = 36)
    public static boolean holdFeedforwardInDeadband = false;
    @Sorter(sort = 37)
    public static boolean profileConfigured = false;
    @Sorter(sort = 38)
    public static boolean profiling = false;
    @Sorter(sort = 39)
    public static boolean brakingForReprofile = false;
    @Sorter(sort = 40)
    public static boolean onTarget = false;
    @Sorter(sort = 41)
    public static boolean settled = false;
    @Sorter(sort = 42)
    public static boolean overCurrent = false;
    @Sorter(sort = 43)
    public static boolean resetIntegralOnSignChange = false;

    @Sorter(sort = 44)
    public static int pidResetCount = 0;
}
