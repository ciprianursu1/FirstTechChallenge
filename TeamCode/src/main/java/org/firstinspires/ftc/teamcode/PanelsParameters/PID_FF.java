package org.firstinspires.ftc.teamcode.PanelsParameters;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

// IMPORTANT : 90% PHYSICS model, 10% PID correction
// PID only corrects for real life inconsistencies, the theoretical is physics
@Configurable
public class PID_FF {
    // ---------------------------------------------------------
    // PID Gains
    // ---------------------------------------------------------
    @Sorter(sort = 1)
    public static double kP = 0.0; // Proportional gain
    @Sorter(sort = 2)
    public static double kI = 0.0; // Integral gain
    @Sorter(sort = 3)
    public static double kD = 0.0; // Derivative gain

    // ---------------------------------------------------------
    // Feedforward Gains
    // ---------------------------------------------------------
    @Sorter(sort = 4)
    public static double kS = 0.0; // Static friction (overcomes stiction)
    @Sorter(sort = 5)
    public static double kV = 0.0; // Velocity feedforward
    @Sorter(sort = 6)
    public static double kA = 0.0; // Acceleration feedforward
    @Sorter(sort = 7)
    public static double kG = 0.0; // Gravity/Cos feedforward (useful for arms/elevators)

    // ---------------------------------------------------------
    // Tolerances & Deadbands
    // ---------------------------------------------------------
    @Sorter(sort = 8)
    public static double deadband = 1.0;        // Strict position error tolerance to output 0
    @Sorter(sort = 9)
    public static double settledDeadband = 2.0; // Looser tolerance to report "isSettled"

    // ---------------------------------------------------------
    // Integral Anti-Windup Limits
    // ---------------------------------------------------------
    @Sorter(sort = 10)
    public static double integralMin = -1.0; // Maximum negative accumulation
    @Sorter(sort = 11)
    public static double integralMax = 1.0;  // Maximum positive accumulation
    @Sorter(sort = 12)
    public static double targetPos = 0.0; // Custom target
    @Sorter(sort = 13)
    public static double targetVel = 0.0;
    @Sorter(sort = 14)
    public static double targetAcc = 0.0;
}