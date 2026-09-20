package org.firstinspires.ftc.teamcode.PanelsParameters;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

@Configurable
public class Arm2DOFTestParameters {
    @Sorter(sort = 1)
    public static String joint1MotorName = "joint1";
    @Sorter(sort = 2)
    public static String joint2MotorName = "joint2";
    @Sorter(sort = 3)
    public static boolean joint1Reversed = false;
    @Sorter(sort = 4)
    public static boolean joint2Reversed = false;
    @Sorter(sort = 5)
    public static boolean resetEncodersOnInit = true;
    @Sorter(sort = 6)
    public static boolean motorOutputEnabled = true;
    @Sorter(sort = 7)
    public static boolean dynamicGravityEnabled = true;
    @Sorter(sort = 8)
    public static boolean holdFeedforwardInDeadband = true;
    @Sorter(sort = 9)
    public static boolean useProfile = false;

    @Sorter(sort = 10)
    public static double joint1TargetDegrees = -4.2401215805;
    @Sorter(sort = 11)
    public static double joint2TargetDegrees = 0.0;
    @Sorter(sort = 12)
    public static double joint1TargetVelocity = 0.0;
    @Sorter(sort = 13)
    public static double joint2TargetVelocity = 0.0;
    @Sorter(sort = 14)
    public static double targetStepDegrees = 2.0;
    @Sorter(sort = 15)
    public static double joint1MaxPower = 0.6;
    @Sorter(sort = 16)
    public static double joint2MaxPower = 0.6;

    @Sorter(sort = 17)
    public static double joint1TicksPerRevolution = 5264.0;
    @Sorter(sort = 18)
    public static double joint2TicksPerRevolution = 5264.0;
    @Sorter(sort = 19)
    public static double joint1HorizontalTicks = -1332.0;
    @Sorter(sort = 20)
    public static double joint2HorizontalTicks = -2372.0;
    @Sorter(sort = 21)
    public static double joint1OutputDirection = 1.0;
    @Sorter(sort = 22)
    public static double joint2OutputDirection = -1.0;

    @Sorter(sort = 23)
    public static boolean joint1LimitsEnabled = true;
    @Sorter(sort = 24)
    public static double joint1MinDegrees = -116.3981762918;
    @Sorter(sort = 25)
    public static double joint1MaxDegrees = -4.2401215805;
    @Sorter(sort = 26)
    public static boolean joint2LimitsEnabled = false;
    @Sorter(sort = 27)
    public static double joint2MinDegrees = -180.0;
    @Sorter(sort = 28)
    public static double joint2MaxDegrees = 180.0;

    @Sorter(sort = 29)
    public static double upperArmMassKg = 0.667;
    @Sorter(sort = 30)
    public static double forearmMassKg = 0.600;
    @Sorter(sort = 31)
    public static double payloadMassKg = 0.0;
    @Sorter(sort = 32)
    public static double upperArmLengthMeters = 0.40;
    @Sorter(sort = 33)
    public static double forearmLengthMeters = 0.340;
    @Sorter(sort = 34)
    public static double upperArmCenterOfMassMeters = 0.35;
    @Sorter(sort = 35)
    public static double forearmCenterOfMassMeters = 0.20;
    @Sorter(sort = 36)
    public static double payloadDistanceFromElbowMeters = 0.340;
    @Sorter(sort = 37)
    public static double joint1HorizontalKg = 0.32;
    @Sorter(sort = 38)
    public static double joint2HorizontalKg = 0.10;

    @Sorter(sort = 39)
    public static double joint1Kp = 0.03;
    @Sorter(sort = 40)
    public static double joint1Ki = 0.004;
    @Sorter(sort = 41)
    public static double joint1Kd = 0.0005;
    @Sorter(sort = 42)
    public static double joint1Ks = 0.002;
    @Sorter(sort = 43)
    public static double joint1Kv = 0.00008;
    @Sorter(sort = 44)
    public static double joint1Ka = 0.00008;
    @Sorter(sort = 45)
    public static double joint1DeadbandDegrees = 2.0;
    @Sorter(sort = 46)
    public static double joint1SettledDegrees = 3.0;
    @Sorter(sort = 47)
    public static double joint1IntegralMin = -100.0;
    @Sorter(sort = 48)
    public static double joint1IntegralMax = 100.0;

    @Sorter(sort = 49)
    public static double joint2Kp = 0.01;
    @Sorter(sort = 50)
    public static double joint2Ki = 0.004;
    @Sorter(sort = 51)
    public static double joint2Kd = 0.0005;
    @Sorter(sort = 52)
    public static double joint2Ks = 0.002;
    @Sorter(sort = 53)
    public static double joint2Kv = 0.0004;
    @Sorter(sort = 54)
    public static double joint2Ka = 0.000002;
    @Sorter(sort = 55)
    public static double joint2DeadbandDegrees = 2.0;
    @Sorter(sort = 56)
    public static double joint2SettledDegrees = 3.0;
    @Sorter(sort = 57)
    public static double joint2IntegralMin = -100.0;
    @Sorter(sort = 58)
    public static double joint2IntegralMax = 100.0;

    @Sorter(sort = 59)
    public static double joint1MaxVelocity = 180.0;
    @Sorter(sort = 60)
    public static double joint1MaxAcceleration = 360.0;
    @Sorter(sort = 61)
    public static double joint1MaxJerk = 1800.0;
    @Sorter(sort = 62)
    public static double joint2MaxVelocity = 180.0;
    @Sorter(sort = 63)
    public static double joint2MaxAcceleration = 360.0;
    @Sorter(sort = 64)
    public static double joint2MaxJerk = 1800.0;

    @Sorter(sort = 65)
    public static boolean enableSlewRate = true;
    @Sorter(sort = 66)
    public static double maxSlewRate = 50.0;
    @Sorter(sort = 67)
    public static boolean enableVoltageCompensation = false;
    @Sorter(sort = 68)
    public static double nominalVoltage = 12.0;
    @Sorter(sort = 69)
    public static boolean enableStallProtection = false;
    @Sorter(sort = 70)
    public static double stallCurrentAmps = 6.5;
    @Sorter(sort = 71)
    public static double stallTimeoutMs = 300.0;
}
