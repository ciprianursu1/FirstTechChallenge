package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config.RobotHardware;
import org.firstinspires.ftc.teamcode.Modules.ArmIK2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TeleOp ORIG Restored", group = "AAAAAAA")
public class TeleOpORIG extends OpMode {
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

    private static final double DISTANCE_PER_ROTATION = 125.66;
    private static final double UP_HEIGHT = 645;
    private static final double DOWN_HEIGHT = 10;
    private static final double MID_HEIGHT = 120;
    private static final double NORMAL_CARTESIAN_SPEED_MPS = 0.20;
    private static final double FINE_CARTESIAN_SPEED_MPS = 0.025;
    private static final double MAX_CONTROL_DT_SECONDS = 0.05;
    private static final double MIN_REACH_METERS = 0.10;
    private static final double MAX_REACH_METERS = L1_METERS + L2_METERS;
    private static final double ARM_PRESET_X_METERS = 0.3;
    private static final double ARM_PRESET_Y_METERS = 0.05;
    private static final double REAR_BOUNDARY_X_METERS = 0.0;
    private static final double POSITION_EPSILON_METERS = 1e-9;
    private static final boolean ELBOW_UP = true;

    private static final double PIVOT_TRAVEL_DEGREES = 350.0;
    private static final double PIVOT_OUTWARD_WORLD_DEGREES = 180.0;
    private static final double PIVOT_ZERO_CLOCKWISE_OFFSET_DEGREES = 7.0;
    private static final double PIVOT_GROUND_WORLD_DEGREES = 270.0;

    private static final double JOINT_TICKS_PER_REVOLUTION = 5264.0;
    private static final double HORIZONTAL_JOINT_1_TICKS = -1332.0;
    private static final double HORIZONTAL_JOINT_2_TICKS = -2372.0;
    private static final double JOINT_1_MIN_HORIZONTAL_TICKS = -370.0;
    private static final double JOINT_1_MAX_HORIZONTAL_TICKS = 1270.0;
    private static final double JOINT_1_MIN_FOLDED_TICKS =
            HORIZONTAL_JOINT_1_TICKS + JOINT_1_MIN_HORIZONTAL_TICKS;
    private static final double JOINT_1_MAX_FOLDED_TICKS =
            HORIZONTAL_JOINT_1_TICKS + JOINT_1_MAX_HORIZONTAL_TICKS;
    private static final double JOINT_1_MIN_MATH_DEGREES =
            JOINT_1_MIN_HORIZONTAL_TICKS * 360.0 / JOINT_TICKS_PER_REVOLUTION;
    private static final double JOINT_1_MAX_MATH_DEGREES =
            JOINT_1_MAX_HORIZONTAL_TICKS * 360.0 / JOINT_TICKS_PER_REVOLUTION;
    private static final double JOINT_1_MIN_OUTPUT_DEGREES =
            JOINT_1_MIN_FOLDED_TICKS * 360.0 / JOINT_TICKS_PER_REVOLUTION;
    private static final double JOINT_1_MAX_OUTPUT_DEGREES =
            JOINT_1_MAX_FOLDED_TICKS * 360.0 / JOINT_TICKS_PER_REVOLUTION;
    private static final double JOINT_1_DIRECTION = 1.0;
    private static final double JOINT_2_DIRECTION = -1.0;
    private static final double JOINT_1_OUTPUT_OFFSET_DEGREES =
            HORIZONTAL_JOINT_1_TICKS * 360.0 / JOINT_TICKS_PER_REVOLUTION;
    private static final double JOINT_2_OUTPUT_OFFSET_DEGREES =
            HORIZONTAL_JOINT_2_TICKS * 360.0 / JOINT_TICKS_PER_REVOLUTION;

    private static final int SLIDER_DOWN = 0;
    private static final int SLIDER_UP = 1;
    private static final int SLIDER_CLEARANCE = 2;
    private static final double CLAW_OPEN_POSITION = 0.2;
    private static final double CLAW_CLOSED_POSITION = 0.6;

    private final ArmIK2D armIk = new ArmIK2D(L1_METERS, L2_METERS);
    private final ArmIK2D.Result armIkResult = new ArmIK2D.Result();
    private final ElapsedTime controlTimer = new ElapsedTime();

    private Follower follower;
    private RobotHardware robotHardware;
    private int sliderPosition = SLIDER_DOWN;
    private double sliderTarget;
    private double sliderOffset0 = 0;
    private double sliderOffset1 = 0;
    private double sliderOffset2 = 0;
    private double joint1Target;
    private double joint2Target;
    private double targetX;
    private double targetY;
    private double shoulderDegrees;
    private double elbowDegrees;
    private double forearmDegrees;
    private double shoulderKg;
    private double elbowKg;
    private double pivotPosition;
    private double pivotTravelDegrees;
    private double elbowTargetX;
    private double chassisSpeedMultiplier = 1.0;
    private boolean targetConstrained;
    private boolean rearBoundarySafe = true;
    private boolean pivotHeadingReachable = true;
    private boolean clawOpen = true;
    private boolean reverseChassis;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        robotHardware = new RobotHardware(hardwareMap);

        robotHardware.cldcjoint1.init(false);
        robotHardware.cldcjoint2.init(false);
        robotHardware.cldcjoint1.enableNonWrappedAngleMode();
        robotHardware.cldcjoint2.enableNonWrappedAngleMode();
        robotHardware.cldcjoint1.disableCosineGravityFeedforward();
        robotHardware.cldcjoint2.disableCosineGravityFeedforward();
        robotHardware.cldcjoint1.enable(true);
        robotHardware.cldcjoint2.enable(true);

        armIk.setOutputDirections(JOINT_1_DIRECTION, JOINT_2_DIRECTION, 1.0);
        armIk.setOutputOffsetsDegrees(
                JOINT_1_OUTPUT_OFFSET_DEGREES,
                JOINT_2_OUTPUT_OFFSET_DEGREES,
                0.0);
        armIk.enableSmallArcShoulderLimit(
                JOINT_1_MIN_OUTPUT_DEGREES,
                JOINT_1_MAX_OUTPUT_DEGREES);
        syncEndEffectorTargetToCurrentArm();
        joint1Target = robotHardware.cldcjoint1.getCurrentPosition();
        joint2Target = robotHardware.cldcjoint2.getCurrentPosition();

        robotHardware.cldcslider.init(true);
        robotHardware.cldcslider.disableAngleLimits();
        robotHardware.cldcslider.enable(false);
    }

    @Override
    public void init_loop() {
        updateArmMotors(false);
        publishTelemetry();
    }

    @Override
    public void start() {
        robotHardware.cldcslider.enable(true);
        follower.startTeleopDrive();
        controlTimer.reset();
    }

    @Override
    public void loop() {
        updateChassis();
        updateEndEffectorTarget();
        updateControls();
        updateMechanisms();
        follower.update();
        publishTelemetry();
    }

    private void updateChassis() {
        chassisSpeedMultiplier = gamepad1.right_trigger > 0.7 ? 0.34 : 1.0;
        if (gamepad1.psWasPressed()) {
            reverseChassis = !reverseChassis;
        }

        if (reverseChassis) {
            follower.setTeleOpDrive(
                    gamepad1.left_stick_y * chassisSpeedMultiplier,
                    gamepad1.left_stick_x * chassisSpeedMultiplier,
                    -gamepad1.right_stick_x * chassisSpeedMultiplier);
        } else {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * chassisSpeedMultiplier,
                    -gamepad1.left_stick_x * chassisSpeedMultiplier,
                    -gamepad1.right_stick_x * chassisSpeedMultiplier);
        }
    }

    private void updateControls() {
        boolean sliderAdjustmentMode =
                gamepad1.left_trigger > 0.5 || gamepad2.left_trigger > 0.5;
        if (sliderAdjustmentMode) {
            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                switch (sliderPosition){
                    case SLIDER_UP:
                        sliderOffset1 -= 3;
                        break;
                    case SLIDER_CLEARANCE:
                        sliderOffset2 -= 2;
                        break;
                    case SLIDER_DOWN:
                        sliderOffset0 -= 1;
                        break;
                }
            }
            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                switch (sliderPosition){
                    case SLIDER_UP:
                        sliderOffset1 += 3;
                        break;
                    case SLIDER_CLEARANCE:
                        sliderOffset2 += 2;
                        break;
                    case SLIDER_DOWN:
                        sliderOffset0 += 1;
                        break;
                }
            }
        } else if (gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed()) {
            sliderPosition = SLIDER_UP;
        } else if (gamepad1.leftBumperWasPressed() || gamepad2.leftBumperWasPressed()) {
            sliderPosition = SLIDER_DOWN;
        }

        if (gamepad1.optionsWasPressed() || gamepad2.optionsWasPressed()) {
            setEndEffectorTarget(ARM_PRESET_X_METERS, ARM_PRESET_Y_METERS);
        }

        if (gamepad1.circleWasPressed() || gamepad2.circleWasPressed()) {
            robotHardware.intake.setPower(1.0);
        } else if (gamepad1.squareWasPressed() || gamepad2.squareWasPressed()) {
            robotHardware.intake.setPower(-1.0);
        } else if (gamepad1.crossWasPressed() || gamepad2.crossWasPressed()) {
            robotHardware.intake.setPower(0.0);
            if (sliderPosition == SLIDER_DOWN) {
                sliderPosition = SLIDER_CLEARANCE;
            }
        }

        if (gamepad1.triangleWasPressed() || gamepad2.triangleWasPressed()) {
            clawOpen = !clawOpen;
        }
    }

    private void updateEndEffectorTarget() {
        double dt = Math.min(controlTimer.seconds(), MAX_CONTROL_DT_SECONDS);
        controlTimer.reset();

        int xDirection = (gamepad1.dpad_left || gamepad2.dpad_left ? 1 : 0)
                - (gamepad1.dpad_right || gamepad2.dpad_right ? 1 : 0);
        int yDirection = (gamepad1.dpad_up || gamepad2.dpad_up ? 1 : 0)
                - (gamepad1.dpad_down || gamepad2.dpad_down ? 1 : 0);
        if (xDirection == 0 && yDirection == 0) {
            targetConstrained = false;
            return;
        }

        double previousX = targetX;
        double previousY = targetY;
        double directionMagnitude = Math.hypot(xDirection, yDirection);
        double speed = gamepad1.left_trigger > 0.25 || gamepad2.left_trigger > 0.25
                ? FINE_CARTESIAN_SPEED_MPS
                : NORMAL_CARTESIAN_SPEED_MPS;
        targetX += (xDirection / directionMagnitude) * speed * dt;
        targetY += (yDirection / directionMagnitude) * speed * dt;
        constrainTargetToWorkspace(xDirection, yDirection, previousX, previousY);
        if (Math.abs(targetX - previousX) > POSITION_EPSILON_METERS
                || Math.abs(targetY - previousY) > POSITION_EPSILON_METERS) {
            if (!updateJointTargets()) {
                targetX = previousX;
                targetY = previousY;
                targetConstrained = true;
                updateJointTargets();
            }
        }
    }

    private void constrainTargetToWorkspace(
            int xDirection,
            int yDirection,
            double previousX,
            double previousY) {
        targetConstrained = false;
        if (!Double.isFinite(targetX) || !Double.isFinite(targetY)) {
            targetX = previousX;
            targetY = previousY;
            targetConstrained = true;
            return;
        }

        if (targetX < REAR_BOUNDARY_X_METERS) {
            targetX = REAR_BOUNDARY_X_METERS;
            targetConstrained = true;
        }

        if (xDirection != 0 && yDirection == 0) {
            targetX = clampHorizontalAtFixedHeight(targetX, targetY, previousX);
        } else if (yDirection != 0 && xDirection == 0) {
            targetY = clampVerticalAtFixedDistance(targetX, targetY, previousY);
        } else {
            clampDiagonalTarget();
        }
    }

    private double clampHorizontalAtFixedHeight(double requestedX, double fixedY, double previousX) {
        if (Math.abs(fixedY) > MAX_REACH_METERS) {
            targetConstrained = true;
            return previousX;
        }

        double maxX = Math.sqrt(
                Math.max(0.0, MAX_REACH_METERS * MAX_REACH_METERS - fixedY * fixedY));
        double constrainedX = Math.min(requestedX, maxX);
        if (Math.hypot(constrainedX, fixedY) < MIN_REACH_METERS) {
            double minX = Math.sqrt(
                    Math.max(0.0, MIN_REACH_METERS * MIN_REACH_METERS - fixedY * fixedY));
            constrainedX = Math.max(constrainedX, minX);
        }
        if (Math.abs(constrainedX - requestedX) > POSITION_EPSILON_METERS) {
            targetConstrained = true;
        }
        return constrainedX;
    }

    private double clampVerticalAtFixedDistance(double fixedX, double requestedY, double previousY) {
        if (fixedX > MAX_REACH_METERS) {
            targetConstrained = true;
            return previousY;
        }

        double maxAbsY = Math.sqrt(
                Math.max(0.0, MAX_REACH_METERS * MAX_REACH_METERS - fixedX * fixedX));
        double constrainedY = Math.max(-maxAbsY, Math.min(maxAbsY, requestedY));
        if (Math.hypot(fixedX, constrainedY) < MIN_REACH_METERS) {
            double minAbsY = Math.sqrt(
                    Math.max(0.0, MIN_REACH_METERS * MIN_REACH_METERS - fixedX * fixedX));
            double side = previousY == 0.0 ? Math.signum(requestedY) : Math.signum(previousY);
            constrainedY = (side == 0.0 ? 1.0 : side) * minAbsY;
        }
        if (Math.abs(constrainedY - requestedY) > POSITION_EPSILON_METERS) {
            targetConstrained = true;
        }
        return constrainedY;
    }

    private void clampDiagonalTarget() {
        double radius = Math.hypot(targetX, targetY);
        double limitedRadius = Math.max(MIN_REACH_METERS, Math.min(MAX_REACH_METERS, radius));
        if (Math.abs(limitedRadius - radius) <= POSITION_EPSILON_METERS) {
            return;
        }

        if (radius > POSITION_EPSILON_METERS) {
            double scale = limitedRadius / radius;
            targetX *= scale;
            targetY *= scale;
        } else {
            targetX = MIN_REACH_METERS;
            targetY = 0.0;
        }
        targetConstrained = true;
    }

    private void updateMechanisms() {
        if (sliderPosition == SLIDER_DOWN) {
            sliderTarget = 384.5 * (DOWN_HEIGHT + sliderOffset0) / DISTANCE_PER_ROTATION;
        } else if (sliderPosition == SLIDER_UP) {
            sliderTarget = 384.5 * (UP_HEIGHT + sliderOffset1) / DISTANCE_PER_ROTATION;
        } else {
            sliderTarget = 384.5 * (MID_HEIGHT + sliderOffset2) / DISTANCE_PER_ROTATION;
        }

        robotHardware.claw.setPosition(
                clawOpen ? CLAW_OPEN_POSITION : CLAW_CLOSED_POSITION);
        updateArmMotors(true);
        robotHardware.cldcslider.update(sliderTarget);
    }

    private boolean updateJointTargets() {
        armIk.solveInto(armIkResult, targetX, targetY, ELBOW_UP, false);
        if (!armIkResult.valid) {
            rearBoundarySafe = false;
            return false;
        }

        elbowTargetX = L1_METERS * Math.cos(Math.toRadians(armIkResult.shoulderDegrees));
        rearBoundarySafe =
                elbowTargetX >= REAR_BOUNDARY_X_METERS - POSITION_EPSILON_METERS
                        && armIkResult.solvedX
                                >= REAR_BOUNDARY_X_METERS - POSITION_EPSILON_METERS;
        if (!rearBoundarySafe) {
            return false;
        }

        joint1Target = armIkResult.shoulderTargetDegrees;
        joint2Target = armIkResult.elbowTargetDegrees;
        return true;
    }

    private void setEndEffectorTarget(double x, double y) {
        double previousX = targetX;
        double previousY = targetY;
        targetX = x;
        targetY = y;
        targetConstrained = false;
        if (!updateJointTargets()) {
            targetX = previousX;
            targetY = previousY;
            targetConstrained = true;
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
        double targetShoulderDegrees = Math.max(
                JOINT_1_MIN_MATH_DEGREES,
                Math.min(JOINT_1_MAX_MATH_DEGREES, shoulderDegrees));
        double targetForearmDegrees = targetShoulderDegrees + elbowDegrees;
        double shoulderRadians = Math.toRadians(targetShoulderDegrees);
        double forearmRadians = Math.toRadians(targetForearmDegrees);
        targetX = (L1_METERS * Math.cos(shoulderRadians))
                + (L2_METERS * Math.cos(forearmRadians));
        targetY = (L1_METERS * Math.sin(shoulderRadians))
                + (L2_METERS * Math.sin(forearmRadians));
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
        robotHardware.cldcjoint2.updateDirect(joint2Target, 0.0);
        robotHardware.cldcjoint1.updateDirect(joint1Target, 0.0);
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

    private void publishTelemetry() {
        telemetry.addData("Arm targets", "%.1f / %.1f deg", joint1Target, joint2Target);
        telemetry.addData("Arm math", "%.1f / %.1f / %.1f deg",
                shoulderDegrees, elbowDegrees, forearmDegrees);
        telemetry.addData("Arm kG", "%.3f / %.3f", shoulderKg, elbowKg);
        telemetry.addData("IK target", "%.3f / %.3f m", targetX, targetY);
        telemetry.addData("IK status", armIkResult.status);
        telemetry.addData("IK constrained", targetConstrained);
        telemetry.addData("Rear boundary safe", rearBoundarySafe);
        telemetry.addData("Slider state/target", "%d / %.1f", sliderPosition, sliderTarget);
        telemetry.addData("Pivot", "%.3f | ground reachable: %b",
                pivotPosition, pivotHeadingReachable);
        robotHardware.cldcjoint1.appendTelemetry(telemetry, "joint1");
        robotHardware.cldcjoint2.appendTelemetry(telemetry, "joint2");
        robotHardware.cldcslider.appendTelemetry(telemetry, "slider");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.breakFollowing();
        }
        if (robotHardware == null) {
            return;
        }

        robotHardware.intake.setPower(0.0);
        robotHardware.cldcjoint1.enable(false);
        robotHardware.cldcjoint2.enable(false);
        robotHardware.cldcslider.enable(false);
        robotHardware.cldcjoint1.updateDirect(
                robotHardware.cldcjoint1.getCurrentPosition(), 0.0);
        robotHardware.cldcjoint2.updateDirect(
                robotHardware.cldcjoint2.getCurrentPosition(), 0.0);
        robotHardware.cldcslider.updateDirect(
                robotHardware.cldcslider.getCurrentPosition(), 0.0);
    }
}
