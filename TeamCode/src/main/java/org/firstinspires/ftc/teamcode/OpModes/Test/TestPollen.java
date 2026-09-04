package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config.RobotHardware;
import org.firstinspires.ftc.teamcode.Modules.ArmIK2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TeleOp ORIG", group = "AAAAAAA")
public class TestPollen extends OpMode {
    private static final double UPPER_ARM_MASS_KG = 0.667;
    private static final double FOREARM_MASS_KG = 0.300;
    private static final double L1_METERS = 0.40;
    private static final double L2_METERS = 0.340;
    private static final double C1_METERS = 0.35;
    private static final double C2_METERS = 0.20;
    private static final double SHOULDER_HORIZONTAL_KG = 0.32;
    private static final double ELBOW_HORIZONTAL_KG = 0.10;
    private static final double SHOULDER_UPPER_MOMENT = UPPER_ARM_MASS_KG * C1_METERS;
    private static final double SHOULDER_FOREARM_BASE_MOMENT = FOREARM_MASS_KG * L1_METERS;
    private static final double SHOULDER_FOREARM_COM_MOMENT = FOREARM_MASS_KG * C2_METERS;
    private static final double SHOULDER_TOTAL_MOMENT =
            SHOULDER_UPPER_MOMENT
                    + SHOULDER_FOREARM_BASE_MOMENT
                    + SHOULDER_FOREARM_COM_MOMENT;
    private static final double DISTANCE_PER_ROTATION = 125.66; // mm
    private static final double UP_HEIGHT = 630; // mm
    private static final double DOWN_HEIGHT = 25; // mm
    private static final double NORMAL_CARTESIAN_SPEED_MPS = 0.20;
    private static final double FINE_CARTESIAN_SPEED_MPS = 0.025;
    private static final double ARM_PRESET_X_METERS = 0.3;
    private static final double ARM_PRESET_Y_METERS = 0.05;
    private static final double MAX_CONTROL_DT_SECONDS = 0.05;
    private static final boolean ELBOW_UP = true;
    private static final double PIVOT_TRAVEL_DEGREES = 350.0;
    private static final double PIVOT_OUTWARD_WORLD_DEGREES = 180.0;
    private static final double PIVOT_ZERO_CLOCKWISE_OFFSET_DEGREES = 7.0;
    private static final double PIVOT_GROUND_WORLD_DEGREES = 270.0;

    // Encoder readings reached when moving from reset 0/0 to horizontal maximum extension.
    private static final double JOINT_TICKS_PER_REVOLUTION = 5264.0;
    private static final double HORIZONTAL_JOINT_1_TICKS = 1955.0;
    private static final double HORIZONTAL_JOINT_2_TICKS = 2561.0;
    private static final double JOINT_1_DIRECTION = 1.0;
    private static final double JOINT_2_DIRECTION = -1.0;
    private static final double JOINT_1_OUTPUT_OFFSET_DEGREES =
            HORIZONTAL_JOINT_1_TICKS * 360.0 / JOINT_TICKS_PER_REVOLUTION;
    private static final double JOINT_2_OUTPUT_OFFSET_DEGREES =
            HORIZONTAL_JOINT_2_TICKS * 360.0 / JOINT_TICKS_PER_REVOLUTION;

    private int sliderpos = 0;
    private double j1target = 0;
    private double j2target = 0;
    private double slidertarget = 0;
    private double targetX = 0;
    private double targetY = 0;
    private double shoulderDegrees = 0;
    private double elbowDegrees = 0;
    private double forearmDegrees = 0;
    private double shoulderKg = 0;
    private double elbowKg = 0;
    private double pivotPosition = 0;
    private double pivotTravelDegrees = 0;
    private boolean pivotHeadingReachable = true;
    private RobotHardware robotHardware;
    private Follower follower;
    private final ArmIK2D armIk = new ArmIK2D(L1_METERS, L2_METERS);
    private final ArmIK2D.Result armIkResult = new ArmIK2D.Result();
    private final ElapsedTime controlTimer = new ElapsedTime();
    boolean clawOpen = true;
    final double clawOpenPos = 0.2;
    final double clawClosedPos = 0.6;
    boolean reverseChassis = false;
    double chassisSpeedMulti = 1;
    double sliderOffset = 0;

    @Override
    public void init() {
        robotHardware = new RobotHardware(hardwareMap);

        robotHardware.cldcjoint1.init(true);
        robotHardware.cldcjoint2.init(true);
        robotHardware.cldcjoint1.enableNonWrappedAngleMode();
        robotHardware.cldcjoint2.enableNonWrappedAngleMode();
        robotHardware.cldcjoint2.disableCosineGravityFeedforward();
        robotHardware.cldcjoint1.disableCosineGravityFeedforward();
        robotHardware.cldcjoint1.enable(true);
        robotHardware.cldcjoint2.enable(true);

        armIk.setOutputDirections(JOINT_1_DIRECTION, JOINT_2_DIRECTION, 1.0);
        armIk.setOutputOffsetsDegrees(
                JOINT_1_OUTPUT_OFFSET_DEGREES,
                JOINT_2_OUTPUT_OFFSET_DEGREES,
                0.0);
        syncEndEffectorTargetToCurrentArm();
        j1target = robotHardware.cldcjoint1.getCurrentPosition();
        j2target = robotHardware.cldcjoint2.getCurrentPosition();

        robotHardware.cldcslider.disableAngleLimits();
        robotHardware.cldcslider.enable(false);
        robotHardware.cldcslider.init(true);

        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void init_loop() {
        updateArmMotors(false);
        telemetry.addLine("Encoder 0/0 is the folded start configuration");
        telemetry.addData(
                "Horizontal calibration",
                "J1 %.0f | J2 %.0f ticks",
                HORIZONTAL_JOINT_1_TICKS,
                HORIZONTAL_JOINT_2_TICKS);
        telemetry.addData("J1 hold target", "%.1f deg", j1target);
        telemetry.addData("J2 hold target", "%.1f deg", j2target);
        telemetry.addData("Shoulder kg", "%.3f", shoulderKg);
        telemetry.addData("Elbow kg", "%.3f", elbowKg);
        telemetry.update();
    }

    @Override
    public void start(){
        robotHardware.cldcslider.enable(true);
        follower.startTeleopDrive();
        controlTimer.reset();
    }

    @Override
    public void loop() {
        if(reverseChassis){
            follower.setTeleOpDrive(
                    gamepad1.left_stick_y*chassisSpeedMulti,
                    gamepad1.left_stick_x*chassisSpeedMulti ,
                    -gamepad1.right_stick_x*chassisSpeedMulti
            );
        } else {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y*chassisSpeedMulti,
                    -gamepad1.left_stick_x*chassisSpeedMulti,
                    -gamepad1.right_stick_x*chassisSpeedMulti
            );
        }
        updateEndEffectorTarget();
        boolean sliderAdjustmentMode =
                gamepad1.left_trigger > 0.5 || gamepad2.left_trigger > 0.5;
        if(sliderAdjustmentMode){
            if(gamepad1.left_bumper || gamepad2.left_bumper) sliderOffset -= 5;
            if(gamepad1.right_bumper || gamepad2.right_bumper) sliderOffset += 5;
        } else {
            if (gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed()) {
                sliderpos = 1;
            } else if (gamepad1.leftBumperWasPressed() || gamepad2.leftBumperWasPressed()) {
                sliderpos = 0;
            }
        }
        if (gamepad1.right_trigger > 0.7) {
            chassisSpeedMulti = 0.34;
        } else {
            chassisSpeedMulti = 1;
        }
        if (gamepad1.optionsWasPressed() || gamepad2.optionsWasPressed()) {
            setEndEffectorTarget(ARM_PRESET_X_METERS, ARM_PRESET_Y_METERS);
        }
        if(gamepad1.psWasPressed()){
            reverseChassis = !reverseChassis;
        }
        if(sliderpos == 0){
            slidertarget = 384.5*(DOWN_HEIGHT)/ DISTANCE_PER_ROTATION;
        } else if (sliderpos == 1){
            slidertarget = 384.5*(UP_HEIGHT + sliderOffset)/ DISTANCE_PER_ROTATION;
        } else if (sliderpos == 2){
            slidertarget = 384.5*(DOWN_HEIGHT+50) / DISTANCE_PER_ROTATION;
        }
        if(gamepad1.circleWasPressed() || gamepad2.circleWasPressed()){
            robotHardware.intake.setPower(1);
        }
        else if(gamepad1.squareWasPressed() || gamepad2.squareWasPressed()){
            robotHardware.intake.setPower(-1);
        } else if(gamepad1.crossWasPressed() || gamepad2.crossWasPressed()){
            robotHardware.intake.setPower(0);
            if (sliderpos == 0) {
                sliderpos = 2;
            }
        }
        if(gamepad1.triangleWasPressed() || gamepad2.triangleWasPressed()){
            clawOpen = !clawOpen;
        }
        if(clawOpen){
            robotHardware.claw.setPosition(clawOpenPos);
        } else {
            robotHardware.claw.setPosition(clawClosedPos);
        }

        updateArmMotors();
        robotHardware.cldcslider.update(slidertarget);

        telemetry.addData("J1 target", j1target);
        telemetry.addData("J2 target", j2target);
        telemetry.addData("Shoulder kg", "%.3f", shoulderKg);
        telemetry.addData("Elbow kg", "%.3f", elbowKg);
        telemetry.addData("Shoulder deg", "%.1f", shoulderDegrees);
        telemetry.addData("Elbow relative deg", "%.1f", elbowDegrees);
        telemetry.addData("Forearm abs deg", "%.1f", forearmDegrees);
        telemetry.addData("Pivot position", "%.3f", pivotPosition);
        telemetry.addData("Pivot CCW travel", "%.1f deg", pivotTravelDegrees);
        telemetry.addData("Pivot ground reachable", pivotHeadingReachable);
        telemetry.addData("Arm lengths", "l1 %.2f m | l2 %.2f m", L1_METERS, L2_METERS);
        robotHardware.cldcjoint1.appendTelemetry(telemetry,"j1");
        robotHardware.cldcjoint2.appendTelemetry(telemetry,"j2");
        robotHardware.cldcslider.appendTelemetry(telemetry,"slider");
        telemetry.addData("IK target xy", "%.3f / %.3f m", targetX, targetY);
        telemetry.addData("IK solved xy", "%.3f / %.3f m", armIkResult.solvedX, armIkResult.solvedY);
        telemetry.addData("IK status", armIkResult.status);
        telemetry.addData(
                "Arm speed",
                gamepad1.left_trigger > 0.25 || gamepad2.left_trigger > 0.25
                        ? "fine"
                        : "normal");
        follower.update();
        telemetry.update();
    }

    private void updateEndEffectorTarget() {
        double dt = Math.min(controlTimer.seconds(), MAX_CONTROL_DT_SECONDS);
        controlTimer.reset();

        int xDirection = (gamepad1.dpad_left || gamepad2.dpad_left ? 1 : 0)
                - (gamepad1.dpad_right || gamepad2.dpad_right ? 1 : 0);
        int yDirection = (gamepad1.dpad_up || gamepad2.dpad_up ? 1 : 0)
                - (gamepad1.dpad_down || gamepad2.dpad_down ? 1 : 0);
        if (xDirection == 0 && yDirection == 0) {
            return;
        }

        double directionMagnitude = Math.hypot(xDirection, yDirection);
        double speed = gamepad1.left_trigger > 0.25 || gamepad2.left_trigger > 0.25
                ? FINE_CARTESIAN_SPEED_MPS
                : NORMAL_CARTESIAN_SPEED_MPS;
        targetX += (xDirection / directionMagnitude) * speed * dt;
        targetY += (yDirection / directionMagnitude) * speed * dt;
        updateJointTargets();
    }

    private boolean updateJointTargets() {
        armIk.solveInto(armIkResult, targetX, targetY, ELBOW_UP, false);
        if (!armIkResult.valid) {
            return false;
        }

        j1target = armIkResult.shoulderTargetDegrees;
        j2target = armIkResult.elbowTargetDegrees;
        return true;
    }

    private void setEndEffectorTarget(double x, double y) {
        double previousX = targetX;
        double previousY = targetY;
        targetX = x;
        targetY = y;
        if (!updateJointTargets()) {
            targetX = previousX;
            targetY = previousY;
            updateJointTargets();
        }
    }

    private double getShoulderMathDegrees() {
        return (robotHardware.cldcjoint1.getCurrentPosition() - JOINT_1_OUTPUT_OFFSET_DEGREES)
                / JOINT_1_DIRECTION;
    }

    private double getElbowMathDegrees() {
        return (robotHardware.cldcjoint2.getCurrentPosition() - JOINT_2_OUTPUT_OFFSET_DEGREES)
                / JOINT_2_DIRECTION;
    }

    private void syncEndEffectorTargetToCurrentArm() {
        shoulderDegrees = getShoulderMathDegrees();
        elbowDegrees = getElbowMathDegrees();
        forearmDegrees = shoulderDegrees + elbowDegrees;
        double shoulderRadians = Math.toRadians(shoulderDegrees);
        double forearmRadians = Math.toRadians(forearmDegrees);
        targetX = (L1_METERS * Math.cos(shoulderRadians))
                + (L2_METERS * Math.cos(forearmRadians));
        targetY = (L1_METERS * Math.sin(shoulderRadians))
                + (L2_METERS * Math.sin(forearmRadians));
    }

    private void updateArmMotors() {
        updateArmMotors(true);
    }

    private void updateArmMotors(boolean updateWrist) {
        shoulderDegrees = getShoulderMathDegrees();
        elbowDegrees = getElbowMathDegrees();
        forearmDegrees = shoulderDegrees + elbowDegrees;
        double shoulderRadians = Math.toRadians(shoulderDegrees);
        double forearmRadians = Math.toRadians(forearmDegrees);
        shoulderKg = calculateShoulderKg(shoulderRadians, forearmRadians) * JOINT_1_DIRECTION;
        elbowKg = calculateElbowKg(forearmRadians) * JOINT_2_DIRECTION;
        if (updateWrist) {
            updatePivot();
        }

        robotHardware.cldcjoint1.setGravityFeedforward(shoulderKg);
        robotHardware.cldcjoint2.setGravityFeedforward(elbowKg);
        robotHardware.cldcjoint2.updateDirect(j2target, 0.0);
        robotHardware.cldcjoint1.updateDirect(j1target, 0.0);
    }

    private void updatePivot() {
        double forearmWorldDegrees = PIVOT_OUTWARD_WORLD_DEGREES - forearmDegrees;
        double pivotZeroWorldDegrees =
                forearmWorldDegrees - PIVOT_ZERO_CLOCKWISE_OFFSET_DEGREES;
        double requestedTravelDegrees =
                PIVOT_GROUND_WORLD_DEGREES - pivotZeroWorldDegrees;
        pivotHeadingReachable =
                requestedTravelDegrees >= 0.0 && requestedTravelDegrees <= PIVOT_TRAVEL_DEGREES;
        pivotTravelDegrees =
                Math.max(0.0, Math.min(PIVOT_TRAVEL_DEGREES, requestedTravelDegrees));
        pivotPosition = pivotTravelDegrees / PIVOT_TRAVEL_DEGREES;
        robotHardware.pivot.setPosition(pivotPosition);
    }

    private double calculateShoulderKg(double shoulderRadians, double forearmRadians) {
        return SHOULDER_HORIZONTAL_KG
                * (((SHOULDER_UPPER_MOMENT + SHOULDER_FOREARM_BASE_MOMENT)
                                * Math.cos(shoulderRadians))
                        + (SHOULDER_FOREARM_COM_MOMENT * Math.cos(forearmRadians)))
                / SHOULDER_TOTAL_MOMENT;
    }

    private double calculateElbowKg(double forearmRadians) {
        return ELBOW_HORIZONTAL_KG * Math.cos(forearmRadians);
    }
}
