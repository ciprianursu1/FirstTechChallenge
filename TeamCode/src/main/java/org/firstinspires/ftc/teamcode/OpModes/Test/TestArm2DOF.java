package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.ClosedLoopDC;
import org.firstinspires.ftc.teamcode.Modules.PIDController;
import org.firstinspires.ftc.teamcode.Modules.SafeDcMotor;
import org.firstinspires.ftc.teamcode.PanelsParameters.Arm2DOFTestParameters;
import org.firstinspires.ftc.teamcode.PanelsParameters.Arm2DOFTelemetry;

@TeleOp(name = "Test: 2DOF Arm", group = "Module Tests")
public class TestArm2DOF extends OpMode {
    private static final double EPSILON = 1e-9;

    private SafeDcMotor safeJoint1;
    private SafeDcMotor safeJoint2;
    private PIDController joint1Pid;
    private PIDController joint2Pid;
    private ClosedLoopDC joint1;
    private ClosedLoopDC joint2;
    private TelemetryManager telemetryM;
    private final ElapsedTime runtime = new ElapsedTime();

    private double joint1TicksPerRevolution;
    private double joint2TicksPerRevolution;
    private double joint1Target;
    private double joint2Target;
    private double lastPanelJoint1Target;
    private double lastPanelJoint2Target;
    private double shoulderMathDegrees;
    private double elbowMathDegrees;
    private double forearmAbsoluteDegrees;
    private double shoulderGravityRatio;
    private double elbowGravityRatio;
    private double joint1DynamicKg;
    private double joint2DynamicKg;
    private boolean joint1TargetLimited;
    private boolean joint2TargetLimited;
    private boolean previousDpadUp;
    private boolean previousDpadDown;
    private boolean previousDpadLeft;
    private boolean previousDpadRight;
    private boolean previousA;
    private boolean previousX;
    private boolean previousUseProfile;

    @Override
    public void init() {
        joint1TicksPerRevolution = validTicksPerRevolution(
                Arm2DOFTestParameters.joint1TicksPerRevolution);
        joint2TicksPerRevolution = validTicksPerRevolution(
                Arm2DOFTestParameters.joint2TicksPerRevolution);

        safeJoint1 = new SafeDcMotor(
                hardwareMap,
                Arm2DOFTestParameters.joint1MotorName,
                Arm2DOFTestParameters.joint1Reversed);
        safeJoint2 = new SafeDcMotor(
                hardwareMap,
                Arm2DOFTestParameters.joint2MotorName,
                Arm2DOFTestParameters.joint2Reversed);
        configureSafeMotor(safeJoint1);
        configureSafeMotor(safeJoint2);

        joint1Pid = createJoint1Pid();
        joint2Pid = createJoint2Pid();
        joint1 = new ClosedLoopDC(
                safeJoint1,
                joint1Pid,
                Arm2DOFTestParameters.joint1MaxPower,
                joint1TicksPerRevolution);
        joint2 = new ClosedLoopDC(
                safeJoint2,
                joint2Pid,
                Arm2DOFTestParameters.joint2MaxPower,
                joint2TicksPerRevolution);
        joint1.setTelemetryVerbosity(ClosedLoopDC.TelemetryVerbosity.DEBUG);
        joint2.setTelemetryVerbosity(ClosedLoopDC.TelemetryVerbosity.DEBUG);
        joint1.init(Arm2DOFTestParameters.resetEncodersOnInit);
        joint2.init(Arm2DOFTestParameters.resetEncodersOnInit);
        joint1.enableNonWrappedAngleMode();
        joint2.enableNonWrappedAngleMode();
        joint1.disableCosineGravityFeedforward();
        joint2.disableCosineGravityFeedforward();

        lastPanelJoint1Target = Arm2DOFTestParameters.joint1TargetDegrees;
        lastPanelJoint2Target = Arm2DOFTestParameters.joint2TargetDegrees;
        joint1Target = lastPanelJoint1Target;
        joint2Target = lastPanelJoint2Target;
        limitTargets();
        previousUseProfile = Arm2DOFTestParameters.useProfile;
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void init_loop() {
        updateTargetsFromGamepadAndPanels();
        runArmController();
        publishTelemetry();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        updateTargetsFromGamepadAndPanels();
        runArmController();
        publishTelemetry();
    }

    private PIDController createJoint1Pid() {
        PIDController pid = new PIDController(
                Arm2DOFTestParameters.joint1Kp,
                Arm2DOFTestParameters.joint1Ki,
                Arm2DOFTestParameters.joint1Kd,
                Arm2DOFTestParameters.joint1Ks,
                Arm2DOFTestParameters.joint1Kv,
                Arm2DOFTestParameters.joint1Ka,
                0.0);
        configureJoint1Pid(pid);
        return pid;
    }

    private PIDController createJoint2Pid() {
        PIDController pid = new PIDController(
                Arm2DOFTestParameters.joint2Kp,
                Arm2DOFTestParameters.joint2Ki,
                Arm2DOFTestParameters.joint2Kd,
                Arm2DOFTestParameters.joint2Ks,
                Arm2DOFTestParameters.joint2Kv,
                Arm2DOFTestParameters.joint2Ka,
                0.0);
        configureJoint2Pid(pid);
        return pid;
    }

    private void configureSafeMotor(SafeDcMotor motor) {
        if (Arm2DOFTestParameters.enableSlewRate) {
            motor.enableSlewRateLimiting(Arm2DOFTestParameters.maxSlewRate);
        }
        if (Arm2DOFTestParameters.enableVoltageCompensation) {
            motor.enableVoltageCompensation(Arm2DOFTestParameters.nominalVoltage);
        }
        if (Arm2DOFTestParameters.enableStallProtection) {
            motor.enableStallProtection(
                    Arm2DOFTestParameters.stallCurrentAmps,
                    Arm2DOFTestParameters.stallTimeoutMs);
        }
    }

    private void configureJoint1Pid(PIDController pid) {
        pid.setPIDF(
                Arm2DOFTestParameters.joint1Kp,
                Arm2DOFTestParameters.joint1Ki,
                Arm2DOFTestParameters.joint1Kd,
                Arm2DOFTestParameters.joint1Ks,
                Arm2DOFTestParameters.joint1Kv,
                Arm2DOFTestParameters.joint1Ka,
                0.0);
        pid.setPositionTolerance(Arm2DOFTestParameters.joint1DeadbandDegrees);
        pid.setSettledTolerance(Arm2DOFTestParameters.joint1SettledDegrees);
        pid.setIntegralLimits(
                Arm2DOFTestParameters.joint1IntegralMax,
                Arm2DOFTestParameters.joint1IntegralMin);
        pid.setHoldFeedforwardInDeadband(Arm2DOFTestParameters.holdFeedforwardInDeadband);
    }

    private void configureJoint2Pid(PIDController pid) {
        pid.setPIDF(
                Arm2DOFTestParameters.joint2Kp,
                Arm2DOFTestParameters.joint2Ki,
                Arm2DOFTestParameters.joint2Kd,
                Arm2DOFTestParameters.joint2Ks,
                Arm2DOFTestParameters.joint2Kv,
                Arm2DOFTestParameters.joint2Ka,
                0.0);
        pid.setPositionTolerance(Arm2DOFTestParameters.joint2DeadbandDegrees);
        pid.setSettledTolerance(Arm2DOFTestParameters.joint2SettledDegrees);
        pid.setIntegralLimits(
                Arm2DOFTestParameters.joint2IntegralMax,
                Arm2DOFTestParameters.joint2IntegralMin);
        pid.setHoldFeedforwardInDeadband(Arm2DOFTestParameters.holdFeedforwardInDeadband);
    }

    private void runArmController() {
        configureJoint1Pid(joint1Pid);
        configureJoint2Pid(joint2Pid);
        joint1.setMaxPower(Arm2DOFTestParameters.joint1MaxPower);
        joint2.setMaxPower(Arm2DOFTestParameters.joint2MaxPower);
        joint1.setSCurveConstraints(
                Arm2DOFTestParameters.joint1MaxVelocity,
                Arm2DOFTestParameters.joint1MaxAcceleration,
                Arm2DOFTestParameters.joint1MaxJerk);
        joint2.setSCurveConstraints(
                Arm2DOFTestParameters.joint2MaxVelocity,
                Arm2DOFTestParameters.joint2MaxAcceleration,
                Arm2DOFTestParameters.joint2MaxJerk);
        joint1.enable(Arm2DOFTestParameters.motorOutputEnabled);
        joint2.enable(Arm2DOFTestParameters.motorOutputEnabled);

        if (previousUseProfile != Arm2DOFTestParameters.useProfile) {
            joint1.reset();
            joint2.reset();
            previousUseProfile = Arm2DOFTestParameters.useProfile;
        }

        updateDynamicGravity();
        joint1.setGravityFeedforward(joint1DynamicKg);
        joint2.setGravityFeedforward(joint2DynamicKg);

        if (Arm2DOFTestParameters.useProfile) {
            joint1.update(joint1Target);
            joint2.update(joint2Target);
        } else {
            joint1.updateDirect(joint1Target, Arm2DOFTestParameters.joint1TargetVelocity);
            joint2.updateDirect(joint2Target, Arm2DOFTestParameters.joint2TargetVelocity);
        }
    }

    private void updateDynamicGravity() {
        double joint1HorizontalDegrees =
                Arm2DOFTestParameters.joint1HorizontalTicks
                        * 360.0 / joint1TicksPerRevolution;
        double joint2HorizontalDegrees =
                Arm2DOFTestParameters.joint2HorizontalTicks
                        * 360.0 / joint2TicksPerRevolution;
        shoulderMathDegrees =
                (joint1.getCurrentPosition() - joint1HorizontalDegrees)
                        / signOrOne(Arm2DOFTestParameters.joint1OutputDirection);
        elbowMathDegrees =
                (joint2.getCurrentPosition() - joint2HorizontalDegrees)
                        / signOrOne(Arm2DOFTestParameters.joint2OutputDirection);
        forearmAbsoluteDegrees = shoulderMathDegrees + elbowMathDegrees;

        if (!Arm2DOFTestParameters.dynamicGravityEnabled) {
            shoulderGravityRatio = 0.0;
            elbowGravityRatio = 0.0;
            joint1DynamicKg = 0.0;
            joint2DynamicKg = 0.0;
            return;
        }

        double upperArmMass = nonnegativeFinite(Arm2DOFTestParameters.upperArmMassKg);
        double forearmMass = nonnegativeFinite(Arm2DOFTestParameters.forearmMassKg);
        double payloadMass = nonnegativeFinite(Arm2DOFTestParameters.payloadMassKg);
        double upperArmLength = nonnegativeFinite(Arm2DOFTestParameters.upperArmLengthMeters);
        double upperArmCom = nonnegativeFinite(
                Arm2DOFTestParameters.upperArmCenterOfMassMeters);
        double forearmCom = nonnegativeFinite(
                Arm2DOFTestParameters.forearmCenterOfMassMeters);
        double payloadDistance = nonnegativeFinite(
                Arm2DOFTestParameters.payloadDistanceFromElbowMeters);

        double shoulderLinkMoment =
                (upperArmMass * upperArmCom)
                        + (forearmMass * upperArmLength)
                        + (payloadMass * upperArmLength);
        double forearmMoment =
                (forearmMass * forearmCom)
                        + (payloadMass * payloadDistance);
        double totalShoulderMoment = shoulderLinkMoment + forearmMoment;
        double shoulderRadians = Math.toRadians(shoulderMathDegrees);
        double forearmRadians = Math.toRadians(forearmAbsoluteDegrees);

        shoulderGravityRatio = totalShoulderMoment > EPSILON
                ? ((shoulderLinkMoment * Math.cos(shoulderRadians))
                                + (forearmMoment * Math.cos(forearmRadians)))
                        / totalShoulderMoment
                : 0.0;
        elbowGravityRatio = forearmMoment > EPSILON ? Math.cos(forearmRadians) : 0.0;
        joint1DynamicKg = finiteOrZero(
                Arm2DOFTestParameters.joint1HorizontalKg
                        * signOrOne(Arm2DOFTestParameters.joint1OutputDirection)
                        * shoulderGravityRatio);
        joint2DynamicKg = finiteOrZero(
                Arm2DOFTestParameters.joint2HorizontalKg
                        * signOrOne(Arm2DOFTestParameters.joint2OutputDirection)
                        * elbowGravityRatio);
    }

    private void updateTargetsFromGamepadAndPanels() {
        if (Double.compare(
                        Arm2DOFTestParameters.joint1TargetDegrees,
                        lastPanelJoint1Target)
                != 0) {
            lastPanelJoint1Target = Arm2DOFTestParameters.joint1TargetDegrees;
            joint1Target = lastPanelJoint1Target;
        }
        if (Double.compare(
                        Arm2DOFTestParameters.joint2TargetDegrees,
                        lastPanelJoint2Target)
                != 0) {
            lastPanelJoint2Target = Arm2DOFTestParameters.joint2TargetDegrees;
            joint2Target = lastPanelJoint2Target;
        }

        double step = Math.abs(finiteOrZero(Arm2DOFTestParameters.targetStepDegrees));
        if (gamepad1.dpad_up && !previousDpadUp) {
            joint1Target += step;
        }
        if (gamepad1.dpad_down && !previousDpadDown) {
            joint1Target -= step;
        }
        if (gamepad1.dpad_right && !previousDpadRight) {
            joint2Target += step;
        }
        if (gamepad1.dpad_left && !previousDpadLeft) {
            joint2Target -= step;
        }

        boolean aPressed = gamepad1.a || gamepad1.cross;
        boolean xPressed = gamepad1.x || gamepad1.square;
        if (aPressed && !previousA) {
            joint1Target = Arm2DOFTestParameters.joint1TargetDegrees;
            joint2Target = Arm2DOFTestParameters.joint2TargetDegrees;
        }
        if (xPressed && !previousX) {
            joint1.reset();
            joint2.reset();
            joint1Pid.reset();
            joint2Pid.reset();
        }

        previousDpadUp = gamepad1.dpad_up;
        previousDpadDown = gamepad1.dpad_down;
        previousDpadLeft = gamepad1.dpad_left;
        previousDpadRight = gamepad1.dpad_right;
        previousA = aPressed;
        previousX = xPressed;
        limitTargets();
    }

    private void limitTargets() {
        double requestedJoint1 = joint1Target;
        double requestedJoint2 = joint2Target;
        joint1Target = limitTarget(
                requestedJoint1,
                joint1Target,
                Arm2DOFTestParameters.joint1LimitsEnabled,
                Arm2DOFTestParameters.joint1MinDegrees,
                Arm2DOFTestParameters.joint1MaxDegrees);
        joint2Target = limitTarget(
                requestedJoint2,
                joint2Target,
                Arm2DOFTestParameters.joint2LimitsEnabled,
                Arm2DOFTestParameters.joint2MinDegrees,
                Arm2DOFTestParameters.joint2MaxDegrees);
        joint1TargetLimited = Double.compare(requestedJoint1, joint1Target) != 0;
        joint2TargetLimited = Double.compare(requestedJoint2, joint2Target) != 0;
    }

    private void updateTelemetryFields() {
        Arm2DOFTelemetry.timeSeconds = runtime.seconds();
        Arm2DOFTelemetry.joint1TargetDegrees = joint1Target;
        Arm2DOFTelemetry.joint1PositionDegrees = joint1.getCurrentPosition();
        Arm2DOFTelemetry.joint1ErrorDegrees = joint1.getTargetError();
        Arm2DOFTelemetry.joint1VelocityDegreesPerSecond = joint1.getLastVelocity();
        Arm2DOFTelemetry.joint1Power = joint1.getLastPower();
        Arm2DOFTelemetry.joint1PidOutput = joint1.getLastPidOutput();
        Arm2DOFTelemetry.joint1DynamicKg = joint1DynamicKg;
        Arm2DOFTelemetry.joint1CurrentAmps = safeJoint1.getCurrentAmps();

        Arm2DOFTelemetry.joint2TargetDegrees = joint2Target;
        Arm2DOFTelemetry.joint2PositionDegrees = joint2.getCurrentPosition();
        Arm2DOFTelemetry.joint2ErrorDegrees = joint2.getTargetError();
        Arm2DOFTelemetry.joint2VelocityDegreesPerSecond = joint2.getLastVelocity();
        Arm2DOFTelemetry.joint2Power = joint2.getLastPower();
        Arm2DOFTelemetry.joint2PidOutput = joint2.getLastPidOutput();
        Arm2DOFTelemetry.joint2DynamicKg = joint2DynamicKg;
        Arm2DOFTelemetry.joint2CurrentAmps = safeJoint2.getCurrentAmps();

        Arm2DOFTelemetry.shoulderMathDegrees = shoulderMathDegrees;
        Arm2DOFTelemetry.elbowMathDegrees = elbowMathDegrees;
        Arm2DOFTelemetry.forearmAbsoluteDegrees = forearmAbsoluteDegrees;
        Arm2DOFTelemetry.shoulderGravityRatio = shoulderGravityRatio;
        Arm2DOFTelemetry.elbowGravityRatio = elbowGravityRatio;
        Arm2DOFTelemetry.joint1TargetLimited = joint1TargetLimited;
        Arm2DOFTelemetry.joint2TargetLimited = joint2TargetLimited;
        Arm2DOFTelemetry.joint1Settled = joint1.isSettled();
        Arm2DOFTelemetry.joint2Settled = joint2.isSettled();
        Arm2DOFTelemetry.joint1OverCurrent = safeJoint1.isOverCurrent();
        Arm2DOFTelemetry.joint2OverCurrent = safeJoint2.isOverCurrent();
        Arm2DOFTelemetry.profiling = joint1.isProfiling() || joint2.isProfiling();
    }

    private void publishTelemetry() {
        updateTelemetryFields();

        telemetry.addLine("2DOF Arm Closed-Loop Test");
        telemetry.addData("Mode", Arm2DOFTestParameters.useProfile ? "Profiled" : "Direct");
        telemetry.addData("Controls", "D-pad U/D: J1 | L/R: J2 | Cross: panel targets | Square: reset PID");
        telemetry.addData(
                "Math angles",
                "shoulder %.1f | elbow %.1f | forearm %.1f deg",
                shoulderMathDegrees,
                elbowMathDegrees,
                forearmAbsoluteDegrees);
        telemetry.addData(
                "Dynamic kG",
                "J1 %.4f | J2 %.4f",
                joint1DynamicKg,
                joint2DynamicKg);
        telemetry.addData(
                "Gravity ratio",
                "J1 %.3f | J2 %.3f",
                shoulderGravityRatio,
                elbowGravityRatio);
        telemetry.addData(
                "Targets limited",
                "J1 %b | J2 %b",
                joint1TargetLimited,
                joint2TargetLimited);
        joint1.appendTelemetry(telemetry, "joint1");
        joint2.appendTelemetry(telemetry, "joint2");

        if (telemetryM == null) {
            telemetry.update();
            return;
        }
        telemetryM.addData("Arm Time", Arm2DOFTelemetry.timeSeconds);
        telemetryM.addData("J1 Target", Arm2DOFTelemetry.joint1TargetDegrees);
        telemetryM.addData("J1 Position", Arm2DOFTelemetry.joint1PositionDegrees);
        telemetryM.addData("J1 Error", Arm2DOFTelemetry.joint1ErrorDegrees);
        telemetryM.addData("J1 Velocity", Arm2DOFTelemetry.joint1VelocityDegreesPerSecond);
        telemetryM.addData("J1 Power", Arm2DOFTelemetry.joint1Power);
        telemetryM.addData("J1 PID Output", Arm2DOFTelemetry.joint1PidOutput);
        telemetryM.addData("J1 Dynamic kG", Arm2DOFTelemetry.joint1DynamicKg);
        telemetryM.addData("J1 Current", Arm2DOFTelemetry.joint1CurrentAmps);
        telemetryM.addData("J2 Target", Arm2DOFTelemetry.joint2TargetDegrees);
        telemetryM.addData("J2 Position", Arm2DOFTelemetry.joint2PositionDegrees);
        telemetryM.addData("J2 Error", Arm2DOFTelemetry.joint2ErrorDegrees);
        telemetryM.addData("J2 Velocity", Arm2DOFTelemetry.joint2VelocityDegreesPerSecond);
        telemetryM.addData("J2 Power", Arm2DOFTelemetry.joint2Power);
        telemetryM.addData("J2 PID Output", Arm2DOFTelemetry.joint2PidOutput);
        telemetryM.addData("J2 Dynamic kG", Arm2DOFTelemetry.joint2DynamicKg);
        telemetryM.addData("J2 Current", Arm2DOFTelemetry.joint2CurrentAmps);
        telemetryM.addData("Shoulder Math Deg", Arm2DOFTelemetry.shoulderMathDegrees);
        telemetryM.addData("Elbow Math Deg", Arm2DOFTelemetry.elbowMathDegrees);
        telemetryM.addData("Forearm Absolute Deg", Arm2DOFTelemetry.forearmAbsoluteDegrees);
        telemetryM.addData("J1 Gravity Ratio", Arm2DOFTelemetry.shoulderGravityRatio);
        telemetryM.addData("J2 Gravity Ratio", Arm2DOFTelemetry.elbowGravityRatio);
        telemetryM.addData("J1 Target Limited", Arm2DOFTelemetry.joint1TargetLimited);
        telemetryM.addData("J2 Target Limited", Arm2DOFTelemetry.joint2TargetLimited);
        telemetryM.addData("J1 Settled", Arm2DOFTelemetry.joint1Settled);
        telemetryM.addData("J2 Settled", Arm2DOFTelemetry.joint2Settled);
        telemetryM.addData("J1 Over Current", Arm2DOFTelemetry.joint1OverCurrent);
        telemetryM.addData("J2 Over Current", Arm2DOFTelemetry.joint2OverCurrent);
        telemetryM.addData("Arm Profiling", Arm2DOFTelemetry.profiling);
        telemetryM.update(telemetry);
    }

    private static double limitTarget(
            double requested,
            double fallback,
            boolean limitsEnabled,
            double firstLimit,
            double secondLimit) {
        if (!Double.isFinite(requested)) {
            return Double.isFinite(fallback) ? fallback : 0.0;
        }
        if (!limitsEnabled || !Double.isFinite(firstLimit) || !Double.isFinite(secondLimit)) {
            return requested;
        }
        double min = Math.min(firstLimit, secondLimit);
        double max = Math.max(firstLimit, secondLimit);
        return Math.max(min, Math.min(max, requested));
    }

    private static double validTicksPerRevolution(double ticksPerRevolution) {
        return Double.isFinite(ticksPerRevolution) && Math.abs(ticksPerRevolution) > EPSILON
                ? Math.abs(ticksPerRevolution)
                : 1.0;
    }

    private static double nonnegativeFinite(double value) {
        return Double.isFinite(value) ? Math.max(0.0, value) : 0.0;
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }

    private static double signOrOne(double value) {
        double sign = Math.signum(value);
        return sign == 0.0 ? 1.0 : sign;
    }

    @Override
    public void stop() {
        if (joint1 != null) {
            joint1.enable(false);
            joint1.updateDirect(joint1.getCurrentPosition(), 0.0);
        }
        if (joint2 != null) {
            joint2.enable(false);
            joint2.updateDirect(joint2.getCurrentPosition(), 0.0);
        }
    }
}
