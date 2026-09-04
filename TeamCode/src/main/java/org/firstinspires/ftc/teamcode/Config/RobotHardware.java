package org.firstinspires.ftc.teamcode.Config;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.ClosedLoopDC;
import org.firstinspires.ftc.teamcode.Modules.PIDController;
import org.firstinspires.ftc.teamcode.Modules.SafeDcMotor;

public class RobotHardware {
    HardwareMap hardwareMap;
    public RobotHardware(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        joint1 = new SafeDcMotor(hardwareMap,"joint1");
        joint2 = new SafeDcMotor(hardwareMap,"joint2");
        slider = new SafeDcMotor(hardwareMap, "slider",true);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        joint1.enableSlewRateLimiting(50);
        joint2.enableSlewRateLimiting(50);
        slider.enableSlewRateLimiting(50);
        pidslider.setSettledTolerance(5);
        pidslider.setPositionTolerance(3);
        pidslider.setHoldFeedforwardInDeadband(true);
        pidslider.setIntegralLimits(200,-200);
        pidjoint1.setSettledTolerance(3);
        pidjoint2.setSettledTolerance(3);
        pidjoint2.setHoldFeedforwardInDeadband(true);
        pidjoint1.setHoldFeedforwardInDeadband(true);
        pidjoint1.setPositionTolerance(2);
        pidjoint2.setPositionTolerance(2);
        pidjoint1.setIntegralLimits(100,-100);
        pidjoint2.setIntegralLimits(100,-100);
        cldcjoint1 = new ClosedLoopDC(joint1,pidjoint1,maxpjoint1,tprjoint1);
        cldcjoint2 = new ClosedLoopDC(joint2,pidjoint2,maxpjoint2,tprjoint2);
        cldcslider = new ClosedLoopDC(slider,pidslider,maxpslider,tprslider);
        pivot = hardwareMap.get(Servo.class,"pivot");
        claw = hardwareMap.get(Servo.class,"claw");
        claw.setDirection(Servo.Direction.REVERSE);
        claw.setPosition(0);
        pivot.setPosition(0);
    }
    SafeDcMotor joint1;
    SafeDcMotor slider;
    public Servo pivot;
    public Servo claw;
    double openposclaw = 120/350.0;
    PIDController pidjoint1 = new PIDController( 0.03,0.005,0.0005,0.02,0.00008,0.00008,0.25);
    int tprjoint1 = 5264;
    double maxpjoint1 = 0.8;
    public ClosedLoopDC cldcjoint1;
    SafeDcMotor joint2;
    PIDController pidjoint2 = new PIDController(0.03,0.004,0.0005,0.002,0.0004,0.0002,0.1);
    PIDController pidslider = new PIDController(0.008,0.004,0.0005,0.002,0.0004,0.00004, 0.18);
    double tprslider = 384.5;
    int tprjoint2 = 5264;
    double maxpjoint2 = 0.8;
    double maxpslider = 0.7;
    public ClosedLoopDC cldcjoint2;
    public ClosedLoopDC cldcslider;
    public DcMotorEx intake;

}
