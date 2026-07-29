package org.firstinspires.ftc.teamcode.PanelsParameters;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

@Configurable
public class SysIdParameters {
    @Sorter(sort = 1)
    public static String motorName = "armMotor";

    @Sorter(sort = 2)
    public static double characterizationPowerSign = 1.0;
    @Sorter(sort = 3)
    public static double encoderVelocitySign = 1.0;
    @Sorter(sort = 4)
    public static double gravityPowerSign = 1.0;

    @Sorter(sort = 5)
    public static boolean armMode = true;
    @Sorter(sort = 6)
    public static double ticksPerOutputRev = 384.5;
    @Sorter(sort = 7)
    public static double horizontalTicks = 0.0;

    @Sorter(sort = 8)
    public static double knownKS = 0.0;
    @Sorter(sort = 9)
    public static double knownKV = 0.0;
    @Sorter(sort = 10)
    public static double knownKG = 0.0;

    @Sorter(sort = 11)
    public static double rampRatePerSec = 0.05;
    @Sorter(sort = 12)
    public static double maxRampPower = 1.0;
    @Sorter(sort = 13)
    public static double startThresholdVelocity = 10.0;

    @Sorter(sort = 14)
    public static double stepPower = 0.70;
    @Sorter(sort = 15)
    public static double stepTestDuration = 1.5;
    @Sorter(sort = 16)
    public static double accelerationThreshold = 100.0;
    @Sorter(sort = 17)
    public static double accelerationFilterAlpha = 0.3;

    @Sorter(sort = 18)
    public static double kgTestDuration = 5.0;
    @Sorter(sort = 19)
    public static double maxHoldPower = 0.7;
    @Sorter(sort = 20)
    public static double holdAdjustRate = 0.00012;
    @Sorter(sort = 21)
    public static double holdVelocityDeadband = 8.0;
    @Sorter(sort = 22)
    public static double minCosineMagnitude = 0.25;

    public static double getAngleRadians(double encoderTicks) {
        return ((encoderTicks - horizontalTicks) / ticksPerOutputRev) * 2.0 * Math.PI;
    }

    public static double getGravityFeedforward(double encoderTicks) {
        if (!armMode) {
            return knownKG * gravityPowerSign;
        }
        return knownKG * gravityPowerSign * Math.cos(getAngleRadians(encoderTicks));
    }

    public static double clampMotorPower(double power) {
        return Math.max(-1.0, Math.min(1.0, power));
    }
}
