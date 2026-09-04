package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.ClosedLoopDC;
import org.firstinspires.ftc.teamcode.Modules.PIDController;
import org.firstinspires.ftc.teamcode.Modules.SafeDcMotor;
import org.firstinspires.ftc.teamcode.PanelsParameters.ClosedLoopDCTelemetry;
import org.firstinspires.ftc.teamcode.PanelsParameters.ModuleTestParameters;
import org.firstinspires.ftc.teamcode.PanelsParameters.PID_FF;

@TeleOp(name = "Test: ClosedLoopDC", group = "Module Tests")
public class TestClosedLoopDC extends OpMode {
    private ClosedLoopDC motor;
    private PIDController pid;
    private SafeDcMotor safeMotor;
    private TelemetryManager telemetryM;
    private final ElapsedTime runtime = new ElapsedTime();
    private double targetPosition;
    private boolean previousDpadUp;
    private boolean previousDpadDown;
    private boolean previousA;
    private boolean previousX;

    @Override
    public void init() {
        safeMotor = new SafeDcMotor(hardwareMap, ModuleTestParameters.motorName);
        if (ModuleTestParameters.enableSlewRate) {
            safeMotor.enableSlewRateLimiting(ModuleTestParameters.maxSlewRate);
        }
        if (ModuleTestParameters.enableVoltageCompensation) {
            safeMotor.enableVoltageCompensation(ModuleTestParameters.nominalVoltage);
        }
        if (ModuleTestParameters.enableStallProtection) {
            safeMotor.enableStallProtection(
                    ModuleTestParameters.stallCurrentAmps,
                    ModuleTestParameters.stallTimeoutMs);
        }

        pid = new PIDController(PID_FF.kP, PID_FF.kI, PID_FF.kD, PID_FF.kS, PID_FF.kV, PID_FF.kA, PID_FF.kG);
        motor = new ClosedLoopDC(safeMotor, pid, ModuleTestParameters.maxPower, ModuleTestParameters.ticksPerRev);
        motor.setTelemetryVerbosity(ClosedLoopDC.TelemetryVerbosity.DEBUG);
        motor.init(ModuleTestParameters.resetEncoderOnInit);
        motor.setAngleMode(ModuleTestParameters.angleMode, ModuleTestParameters.angleWrapping);
        updateAngleLimits();
        updateGravityMode();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        targetPosition = ModuleTestParameters.targetPosition;
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        updateTargetFromGamepad();

        pid.setPIDF(PID_FF.kP, PID_FF.kI, PID_FF.kD, PID_FF.kS, PID_FF.kV, PID_FF.kA, PID_FF.kG);
        pid.setPositionTolerance(PID_FF.deadband);
        pid.setSettledTolerance(PID_FF.settledDeadband);
        pid.setIntegralLimits(PID_FF.integralMax, PID_FF.integralMin);
        pid.setHoldFeedforwardInDeadband(ModuleTestParameters.holdFeedforwardInDeadband);

        motor.setMaxPower(ModuleTestParameters.maxPower);
        motor.setAngleMode(ModuleTestParameters.angleMode, ModuleTestParameters.angleWrapping);
        updateAngleLimits();
        updateGravityMode();
        motor.setSCurveConstraints(
                ModuleTestParameters.maxVel,
                ModuleTestParameters.maxAccel,
                ModuleTestParameters.maxJerk);

        if (ModuleTestParameters.useProfile) {
            motor.update(targetPosition);
        } else {
            motor.updateDirect(targetPosition, PID_FF.targetVel);
        }

        updatePanelGraphFields();
        publishDriverStationTelemetry();
        publishPanelsTelemetry();
    }

    private void updatePanelGraphFields() {
        ClosedLoopDCTelemetry.timeSeconds = runtime.seconds();
        ClosedLoopDCTelemetry.target = targetPosition;
        ClosedLoopDCTelemetry.position = motor.getCurrentPosition();
        ClosedLoopDCTelemetry.error = motor.getTargetError();
        ClosedLoopDCTelemetry.velocity = motor.getLastVelocity();
        ClosedLoopDCTelemetry.commandedPower = motor.getLastPower();
        ClosedLoopDCTelemetry.pidOutput = motor.getLastPidOutput();
        ClosedLoopDCTelemetry.effectiveKg = motor.getLastEffectiveKg();
        ClosedLoopDCTelemetry.motorCurrentAmps = safeMotor.getCurrentAmps();

        ClosedLoopDCTelemetry.maxPower = motor.getMaxPower();
        ClosedLoopDCTelemetry.pidTarget = motor.getLastPidTarget();
        ClosedLoopDCTelemetry.pidCurrent = motor.getLastPidCurrent();
        ClosedLoopDCTelemetry.pidError = pid.getLastError();
        ClosedLoopDCTelemetry.pidErrorSum = pid.getErrorSum();
        ClosedLoopDCTelemetry.pidDt = pid.getLastDt();
        ClosedLoopDCTelemetry.pidIntegralDelta = pid.getLastIntegralDelta();

        ClosedLoopDCTelemetry.kP = pid.getkP();
        ClosedLoopDCTelemetry.kI = pid.getkI();
        ClosedLoopDCTelemetry.kD = pid.getkD();
        ClosedLoopDCTelemetry.kS = pid.getkS();
        ClosedLoopDCTelemetry.kV = pid.getkV();
        ClosedLoopDCTelemetry.kA = pid.getkA();
        ClosedLoopDCTelemetry.kG = pid.getkG();
        ClosedLoopDCTelemetry.deadband = pid.getDeadband();
        ClosedLoopDCTelemetry.settledDeadband = pid.getSettledDeadband();
        ClosedLoopDCTelemetry.integralMin = pid.getIntegralMin();
        ClosedLoopDCTelemetry.integralMax = pid.getIntegralMax();

        ClosedLoopDCTelemetry.profileTarget = motor.getActiveProfileTarget();
        ClosedLoopDCTelemetry.profileTimeSeconds = motor.getProfileTime();
        ClosedLoopDCTelemetry.profileTotalTimeSeconds = motor.getProfileTotalTime();
        ClosedLoopDCTelemetry.gravityAngleDegrees = motor.getGravityAngleDegrees();
        ClosedLoopDCTelemetry.horizontalTicks = motor.getGravityHorizontalTicks();
        ClosedLoopDCTelemetry.limitedTarget = motor.getLastLimitedTarget();
        ClosedLoopDCTelemetry.angleLimitA = motor.getAngleLimitA();
        ClosedLoopDCTelemetry.angleLimitB = motor.getAngleLimitB();
        ClosedLoopDCTelemetry.angleLimitArcStart = motor.getAngleLimitArcStart();
        ClosedLoopDCTelemetry.angleLimitArcEnd = motor.getAngleLimitArcEndDegrees();
        ClosedLoopDCTelemetry.angleLimitArcLength = motor.getAngleLimitArcLength();

        ClosedLoopDCTelemetry.enabled = motor.isEnabled();
        ClosedLoopDCTelemetry.angleMode = ModuleTestParameters.angleMode;
        ClosedLoopDCTelemetry.angleWrapping = motor.isAngleWrappingEnabled();
        ClosedLoopDCTelemetry.angleLimits = motor.areAngleLimitsEnabled();
        ClosedLoopDCTelemetry.largeArcAngleLimit = motor.isLargeArcAngleLimit();
        ClosedLoopDCTelemetry.cosineGravity = motor.isCosineGravityEnabled();
        ClosedLoopDCTelemetry.holdFeedforwardInDeadband = pid.getHoldFeedforwardInDeadband();
        ClosedLoopDCTelemetry.profileConfigured = motor.isProfileConfigured();
        ClosedLoopDCTelemetry.profiling = motor.isProfiling();
        ClosedLoopDCTelemetry.brakingForReprofile = motor.isBrakingForReprofile();
        ClosedLoopDCTelemetry.onTarget = motor.isOnTarget();
        ClosedLoopDCTelemetry.settled = motor.isSettled();
        ClosedLoopDCTelemetry.overCurrent = safeMotor.isOverCurrent();
        ClosedLoopDCTelemetry.resetIntegralOnSignChange = pid.getResetIntegralOnSignChange();
        ClosedLoopDCTelemetry.pidResetCount = pid.getResetCount();
    }

    private void publishPanelsTelemetry() {
        if (telemetryM == null) return;

        telemetryM.addData("CLDC Time", ClosedLoopDCTelemetry.timeSeconds);
        telemetryM.addData("CLDC Target", ClosedLoopDCTelemetry.target);
        telemetryM.addData("CLDC Position", ClosedLoopDCTelemetry.position);
        telemetryM.addData("CLDC Error", ClosedLoopDCTelemetry.error);
        telemetryM.addData("CLDC Velocity", ClosedLoopDCTelemetry.velocity);
        telemetryM.addData("CLDC Power", ClosedLoopDCTelemetry.commandedPower);
        telemetryM.addData("CLDC PID Output", ClosedLoopDCTelemetry.pidOutput);
        telemetryM.addData("CLDC Effective kG", ClosedLoopDCTelemetry.effectiveKg);
        telemetryM.addData("CLDC Current", ClosedLoopDCTelemetry.motorCurrentAmps);
        telemetryM.addData("CLDC Max Power", ClosedLoopDCTelemetry.maxPower);
        telemetryM.addData("CLDC PID Target", ClosedLoopDCTelemetry.pidTarget);
        telemetryM.addData("CLDC PID Current", ClosedLoopDCTelemetry.pidCurrent);
        telemetryM.addData("CLDC PID Error", ClosedLoopDCTelemetry.pidError);
        telemetryM.addData("CLDC PID Error Sum", ClosedLoopDCTelemetry.pidErrorSum);
        telemetryM.addData("CLDC PID dt", ClosedLoopDCTelemetry.pidDt);
        telemetryM.addData("CLDC Integral Delta", ClosedLoopDCTelemetry.pidIntegralDelta);
        telemetryM.addData("CLDC kP", ClosedLoopDCTelemetry.kP);
        telemetryM.addData("CLDC kI", ClosedLoopDCTelemetry.kI);
        telemetryM.addData("CLDC kD", ClosedLoopDCTelemetry.kD);
        telemetryM.addData("CLDC kS", ClosedLoopDCTelemetry.kS);
        telemetryM.addData("CLDC kV", ClosedLoopDCTelemetry.kV);
        telemetryM.addData("CLDC kA", ClosedLoopDCTelemetry.kA);
        telemetryM.addData("CLDC kG", ClosedLoopDCTelemetry.kG);
        telemetryM.addData("CLDC Deadband", ClosedLoopDCTelemetry.deadband);
        telemetryM.addData("CLDC Settled Deadband", ClosedLoopDCTelemetry.settledDeadband);
        telemetryM.addData("CLDC Integral Min", ClosedLoopDCTelemetry.integralMin);
        telemetryM.addData("CLDC Integral Max", ClosedLoopDCTelemetry.integralMax);
        telemetryM.addData("CLDC Profile Target", ClosedLoopDCTelemetry.profileTarget);
        telemetryM.addData("CLDC Profile Time", ClosedLoopDCTelemetry.profileTimeSeconds);
        telemetryM.addData("CLDC Profile Total", ClosedLoopDCTelemetry.profileTotalTimeSeconds);
        telemetryM.addData("CLDC Gravity Angle", ClosedLoopDCTelemetry.gravityAngleDegrees);
        telemetryM.addData("CLDC Horizontal Ticks", ClosedLoopDCTelemetry.horizontalTicks);
        telemetryM.addData("CLDC Limited Target", ClosedLoopDCTelemetry.limitedTarget);
        telemetryM.addData("CLDC Angle Limit A", ClosedLoopDCTelemetry.angleLimitA);
        telemetryM.addData("CLDC Angle Limit B", ClosedLoopDCTelemetry.angleLimitB);
        telemetryM.addData("CLDC Limit Arc Start", ClosedLoopDCTelemetry.angleLimitArcStart);
        telemetryM.addData("CLDC Limit Arc End", ClosedLoopDCTelemetry.angleLimitArcEnd);
        telemetryM.addData("CLDC Limit Arc Length", ClosedLoopDCTelemetry.angleLimitArcLength);
        telemetryM.addData("CLDC Enabled", ClosedLoopDCTelemetry.enabled);
        telemetryM.addData("CLDC Angle Mode", ClosedLoopDCTelemetry.angleMode);
        telemetryM.addData("CLDC Angle Wrap", ClosedLoopDCTelemetry.angleWrapping);
        telemetryM.addData("CLDC Angle Limits", ClosedLoopDCTelemetry.angleLimits);
        telemetryM.addData("CLDC Large Arc Limit", ClosedLoopDCTelemetry.largeArcAngleLimit);
        telemetryM.addData("CLDC Cosine Gravity", ClosedLoopDCTelemetry.cosineGravity);
        telemetryM.addData("CLDC Hold FF Deadband", ClosedLoopDCTelemetry.holdFeedforwardInDeadband);
        telemetryM.addData("CLDC Profile Configured", ClosedLoopDCTelemetry.profileConfigured);
        telemetryM.addData("CLDC Profiling", ClosedLoopDCTelemetry.profiling);
        telemetryM.addData("CLDC Reprofile Brake", ClosedLoopDCTelemetry.brakingForReprofile);
        telemetryM.addData("CLDC On Target", ClosedLoopDCTelemetry.onTarget);
        telemetryM.addData("CLDC Settled", ClosedLoopDCTelemetry.settled);
        telemetryM.addData("CLDC Over Current", ClosedLoopDCTelemetry.overCurrent);
        telemetryM.addData("CLDC Reset Integral Sign", ClosedLoopDCTelemetry.resetIntegralOnSignChange);
        telemetryM.addData("CLDC PID Resets", ClosedLoopDCTelemetry.pidResetCount);
        telemetryM.update(telemetry);
    }

    private void publishDriverStationTelemetry() {
        telemetry.addLine("ClosedLoopDC Test");
        telemetry.addData("Mode", ModuleTestParameters.useProfile ? "Profiled" : "Direct");
        telemetry.addData("Dashboard Target", "%.1f", ModuleTestParameters.targetPosition);
        telemetry.addData("Active Target", "%.1f", targetPosition);
        telemetry.addData("Configured Profile", motor.isProfileConfigured());
        telemetry.addData("Angle Wrap", motor.isAngleWrappingEnabled());
        telemetry.addData("Angle Limits", "%b | Large Arc: %b",
                motor.areAngleLimitsEnabled(), motor.isLargeArcAngleLimit());
        telemetry.addData("Limited Target", "%.1f", motor.getLastLimitedTarget());
        telemetry.addData("Cosine Gravity", motor.isCosineGravityEnabled());
        telemetry.addData("Hold FF Deadband", pid.getHoldFeedforwardInDeadband());
        telemetry.addData("Effective kG", "%.4f", motor.getLastEffectiveKg());
        motor.appendTelemetry(telemetry, ModuleTestParameters.motorName);
    }

    private void updateAngleLimits() {
        if (ModuleTestParameters.enableAngleLimits) {
            motor.setAngleLimits(
                    ModuleTestParameters.firstAngleLimitDegrees,
                    ModuleTestParameters.secondAngleLimitDegrees,
                    ModuleTestParameters.useLargeArcAngleLimit);
        } else {
            motor.disableAngleLimits();
        }
    }

    private void updateGravityMode() {
        if (ModuleTestParameters.enableCosineGravity) {
            motor.enableCosineGravityFeedforward(
                    ModuleTestParameters.gravityHorizontalTicks,
                    ModuleTestParameters.gravityPowerSign);
        } else {
            motor.disableCosineGravityFeedforward();
        }
    }

    private void updateTargetFromGamepad() {
        if (ModuleTestParameters.targetPosition != targetPosition
                && !gamepad1.dpad_up
                && !gamepad1.dpad_down) {
            targetPosition = ModuleTestParameters.targetPosition;
        }

        if (gamepad1.dpad_up && !previousDpadUp) {
            targetPosition += ModuleTestParameters.targetStep;
        }
        if (gamepad1.dpad_down && !previousDpadDown) {
            targetPosition -= ModuleTestParameters.targetStep;
        }
        if ((gamepad1.a || gamepad1.cross) && !previousA) {
            targetPosition = ModuleTestParameters.targetPosition;
        }
        if ((gamepad1.x || gamepad1.square) && !previousX) {
            motor.reset();
            pid.reset();
        }

        previousDpadUp = gamepad1.dpad_up;
        previousDpadDown = gamepad1.dpad_down;
        previousA = gamepad1.a || gamepad1.cross;
        previousX = gamepad1.x || gamepad1.square;
    }

    @Override
    public void stop() {
        if (motor != null) {
            motor.enable(false);
            motor.updateDirect(motor.getCurrentPosition(), 0);
        }
    }
}
