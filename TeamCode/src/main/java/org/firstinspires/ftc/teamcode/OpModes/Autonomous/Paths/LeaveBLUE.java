package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config.RobotHardware;
import org.firstinspires.ftc.teamcode.Modules.ArmIK2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Leave BLUE")
public class LeaveBLUE extends OpMode {
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
    private static final double DOWN_HEIGHT = 25;
    private static final double ARM_PRESET_X_METERS = 0.3;
    private static final double ARM_PRESET_Y_METERS = 0.05;
    private static final double REAR_BOUNDARY_X_METERS = 0.0;
    private static final double POSITION_EPSILON_METERS = 1e-9;
    private static final boolean ELBOW_UP = true;
    private static final double SCORE_EJECT_SECONDS = 5.0;
    private static final double SCORE_APPROACH_MAX_POWER = 0.2;

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
    private final ElapsedTime scoreTimer = new ElapsedTime();

    private Follower follower;
    private RobotHardware robotHardware;
    private int stage;
    private boolean pathStarted;
    private boolean stageActionTriggered;
    private boolean scoreWaitStarted;
    private int sliderPosition = SLIDER_DOWN;
    private double sliderTarget;
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
    private double elbowTargetX;
    private boolean targetConstrained;
    private boolean rearBoundarySafe = true;
    private boolean pivotHeadingReachable = true;
    private boolean clawOpen = true;
    public static class Paths {
        public PathChain Path1;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134.000, 82.000),

                                    new Pose(86.000, 85.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
                    .build();

        }
    }
    private Paths paths;
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(134, 82, 180));
        robotHardware = new RobotHardware(hardwareMap);

        robotHardware.cldcjoint1.init(true);
        robotHardware.cldcjoint2.init(true);
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
        updateJointTargets();
        updatePivot();

        robotHardware.cldcslider.disableAngleLimits();
        robotHardware.cldcslider.enable(true);
        robotHardware.cldcslider.init(true);
        paths = new Paths(follower);
    }

    @Override
    public void init_loop() {
        follower.update();
        updateMechanisms();
        publishTelemetry();
    }


    @Override
    public void loop() {
        follower.update();
        updatePathState();
        updateMechanisms();
        publishTelemetry();
    }

    private void updatePathState() {
        switch (stage) {
            case 0:
                if (!pathStarted) {
                    startPath(paths.Path1, 1.0);
                }
                if (pathStarted && !follower.isBusy()) {
                    advanceToStage(1);
                }
                break;
            case 1:
                stop();
                break;
        }
    }

    private void startPath(PathChain path, double maxPower) {
        follower.followPath(path, maxPower, true);
        pathStarted = true;
    }

    private void waitForScoreAndAdvance(int nextStage) {
        if (!robotHardware.cldcslider.isSettled()) {
            return;
        }
        if (!scoreWaitStarted) {
            applySquareAction();
            scoreTimer.reset();
            scoreWaitStarted = true;
            return;
        }
        if (scoreTimer.seconds() >= SCORE_EJECT_SECONDS) {
            advanceToStage(nextStage);
        }
    }

    private void advanceToStage(int nextStage) {
        stage = nextStage;
        pathStarted = false;
        stageActionTriggered = false;
        scoreWaitStarted = false;
    }

    private void applyCrossAction() {
        robotHardware.intake.setPower(0.0);
        if (sliderPosition == SLIDER_DOWN) {
            sliderPosition = SLIDER_CLEARANCE;
        }
    }

    private void applyOptionsAction() {
        setEndEffectorTarget(ARM_PRESET_X_METERS, ARM_PRESET_Y_METERS);
    }

    private void applyLeftBumperAction() {
        sliderPosition = SLIDER_DOWN;
    }

    private void applyRightBumperAction() {
        sliderPosition = SLIDER_UP;
    }

    private void applyCircleAction() {
        robotHardware.intake.setPower(1.0);
    }

    private void applySquareAction() {
        robotHardware.intake.setPower(-1.0);
    }

    private void applyTravelResetActions() {
        applyCrossAction();
        applyLeftBumperAction();
        applyCrossAction();
    }

    private void updateMechanisms() {
        if (sliderPosition == SLIDER_DOWN) {
            sliderTarget = 384.5 * DOWN_HEIGHT / DISTANCE_PER_ROTATION;
        } else if (sliderPosition == SLIDER_UP) {
            sliderTarget = 384.5 * UP_HEIGHT / DISTANCE_PER_ROTATION;
        } else {
            sliderTarget = 384.5 * (DOWN_HEIGHT + 50.0) / DISTANCE_PER_ROTATION;
        }

        robotHardware.claw.setPosition(
                clawOpen ? CLAW_OPEN_POSITION : CLAW_CLOSED_POSITION);
        updateArmMotors();
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
        double targetShoulderDegrees =
                Math.max(
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

    private void updateArmMotors() {
        shoulderDegrees = getShoulderMathDegrees();
        elbowDegrees = getElbowMathDegrees();
        forearmDegrees = shoulderDegrees + elbowDegrees;
        double shoulderRadians = Math.toRadians(shoulderDegrees);
        double forearmRadians = Math.toRadians(forearmDegrees);
        shoulderKg = calculateShoulderKg(shoulderRadians, forearmRadians) * JOINT_1_DIRECTION;
        elbowKg = calculateElbowKg(forearmRadians) * JOINT_2_DIRECTION;
        updatePivot();

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
        double pivotTravelDegrees =
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
        telemetry.addData("Stage", stage);
        telemetry.addData("Path busy", follower.isBusy());
        telemetry.addData("Path completion", "%.3f", follower.getPathCompletion());
        telemetry.addData("Slider state/target", "%d / %.1f", sliderPosition, sliderTarget);
        telemetry.addData("Slider settled", robotHardware.cldcslider.isSettled());
        telemetry.addData("Intake power", "%.1f", robotHardware.intake.getPower());
        telemetry.addData("Score wait", scoreWaitStarted ? scoreTimer.seconds() : 0.0);
        telemetry.addData("Arm target", "%.1f / %.1f deg", joint1Target, joint2Target);
        telemetry.addData(
                "Arm math",
                "%.1f / %.1f / %.1f deg",
                shoulderDegrees,
                elbowDegrees,
                forearmDegrees);
        telemetry.addData("Arm kG", "%.3f / %.3f", shoulderKg, elbowKg);
        telemetry.addData("IK target", "%.3f / %.3f m", targetX, targetY);
        telemetry.addData("IK status", armIkResult.status);
        telemetry.addData("IK constrained", targetConstrained);
        telemetry.addData("Rear boundary safe", rearBoundarySafe);
        telemetry.addData(
                "Pivot",
                "%.3f | ground reachable: %b",
                pivotPosition,
                pivotHeadingReachable);
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
