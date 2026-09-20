package org.firstinspires.ftc.teamcode.Modules;

/**
 * Closed-form inverse kinematics solver for a planar 3-DOF arm.
 *
 * <p>This class solves the common FTC arm layout where all joints move in the same 2D plane:
 * shoulder, elbow, and wrist. The requested end-effector pose is {@code x}, {@code y}, and a final
 * tool angle. The solver first subtracts the wrist/tool length from the target pose to find the
 * wrist center, solves the shoulder and elbow as a two-link arm, then assigns the wrist angle needed
 * to make the end effector match the requested heading.</p>
 *
 * <p>The math is closed-form and does not use iteration. A normal solve performs a fixed number of
 * arithmetic and trigonometric operations, making it reasonable to run every loop on an FTC Control
 * Hub. For loop-time use, prefer {@link #solveInto(Result, double, double, double, boolean, boolean)}
 * with a reused {@link Result} object to avoid repeated allocation.</p>
 *
 * <p>Coordinate convention:</p>
 * <ul>
 *     <li>{@code x} is horizontal distance from the shoulder pivot.</li>
 *     <li>{@code y} is vertical distance from the shoulder pivot.</li>
 *     <li>Positive angles are counter-clockwise in standard math coordinates.</li>
 *     <li>All angles exposed by this class are in degrees and normalized to {@code (-180, 180]}.</li>
 *     <li>All lengths and positions use the same arbitrary linear unit, such as inches or centimeters.</li>
 * </ul>
 *
 * <p>Joint convention:</p>
 * <ul>
 *     <li>{@code shoulderDegrees} is the absolute shoulder link angle in world coordinates.</li>
 *     <li>{@code elbowDegrees} is the relative angle from the shoulder link to the elbow link.</li>
 *     <li>{@code wristDegrees} is the relative angle from the elbow link to the end effector.</li>
 *     <li>{@code shoulderTargetDegrees}, {@code elbowTargetDegrees}, and {@code wristTargetDegrees}
 *     include configured output directions and offsets for use with motor/servo controllers.</li>
 * </ul>
 *
 * <p>Typical FTC use is to feed the target angles into three {@link ClosedLoopDC} instances running
 * in angle mode, or into one or more servos after applying your own scaling.</p>
 */
public class ArmIK2D {
    private static final double EPSILON = 1e-9;
    private static final double DEGREES_TO_RADIANS = Math.PI / 180.0;
    private static final double RADIANS_TO_DEGREES = 180.0 / Math.PI;

    /**
     * Result state returned by a solve.
     *
     * <p>Use this enum to decide whether the arm should execute the returned targets. A result can be
     * useful even when not fully valid. For example, {@link #TARGET_CLAMPED_TO_REACHABLE_RANGE}
     * provides the closest reachable solution when unreachable clamping is enabled.</p>
     */
    public enum Status {
        /** The requested pose was reachable, inside enabled joint limits, and solved normally. */
        VALID,

        /**
         * The requested wrist center was outside the arm reach, but clamping was enabled and the
         * solver returned the closest reachable pose along the same shoulder-to-target ray.
         */
        TARGET_CLAMPED_TO_REACHABLE_RANGE,

        /** One or more link lengths are invalid. Shoulder and elbow must be positive and finite. */
        INVALID_LENGTHS,

        /** The requested {@code x}, {@code y}, or end-effector angle was NaN or infinite. */
        INVALID_TARGET,

        /**
         * The requested wrist center was unreachable and clamping was disabled, so no joint targets
         * should be used.
         */
        UNREACHABLE,

        /**
         * The pose was geometrically solvable, but at least one configured output joint limit rejected
         * the resulting target angle.
         */
        OUTSIDE_JOINT_LIMITS
    }

    /**
     * Mutable output container for one IK solve.
     *
     * <p>The fields are public by design so this object can be reused cheaply inside an FTC loop
     * without getter overhead or new allocations. Treat every field as read-only after calling
     * {@link ArmIK2D#solveInto(Result, double, double, double, boolean, boolean)}.</p>
     */
    public static class Result {
        /** High-level solve result. Check this first when debugging a failed solve. */
        public Status status = Status.INVALID_TARGET;

        /**
         * True when the output targets are safe to use. This is true for {@link Status#VALID} and
         * {@link Status#TARGET_CLAMPED_TO_REACHABLE_RANGE}.
         */
        public boolean valid = false;

        /**
         * True when the original requested wrist center was physically reachable before any optional
         * clamping was applied.
         */
        public boolean reachable = false;

        /**
         * True when the requested wrist center was outside the reachable radius and the solver moved
         * it to the closest reachable radius because clamping was enabled.
         */
        public boolean targetClamped = false;

        /** True when all enabled shoulder, elbow, and wrist output limits accept the solved targets. */
        public boolean withinJointLimits = true;

        /** Original requested end-effector x coordinate. */
        public double requestedX = 0.0;

        /** Original requested end-effector y coordinate. */
        public double requestedY = 0.0;

        /** Original requested end-effector heading in degrees, normalized to {@code (-180, 180]}. */
        public double requestedEndEffectorDegrees = 0.0;

        /**
         * Forward-kinematics x coordinate produced by the solved joint angles. For a valid unclamped
         * solve this should be very close to {@link #requestedX}.
         */
        public double solvedX = 0.0;

        /**
         * Forward-kinematics y coordinate produced by the solved joint angles. For a valid unclamped
         * solve this should be very close to {@link #requestedY}.
         */
        public double solvedY = 0.0;

        /**
         * Wrist-center x coordinate used by the shoulder/elbow two-link solve. This equals the target
         * x minus the wrist/tool vector.
         */
        public double wristCenterX = 0.0;

        /**
         * Wrist-center y coordinate used by the shoulder/elbow two-link solve. This equals the target
         * y minus the wrist/tool vector.
         */
        public double wristCenterY = 0.0;

        /** Raw mathematical shoulder angle in degrees before output offset/direction conversion. */
        public double shoulderDegrees = 0.0;

        /** Raw mathematical elbow angle in degrees before output offset/direction conversion. */
        public double elbowDegrees = 0.0;

        /** Raw mathematical wrist angle in degrees before output offset/direction conversion. */
        public double wristDegrees = 0.0;

        /**
         * Shoulder target angle after applying
         * {@link ArmIK2D#setOutputDirections(double, double, double)} and
         * {@link ArmIK2D#setOutputOffsetsDegrees(double, double, double)}.
         */
        public double shoulderTargetDegrees = 0.0;

        /**
         * Elbow target angle after applying
         * {@link ArmIK2D#setOutputDirections(double, double, double)} and
         * {@link ArmIK2D#setOutputOffsetsDegrees(double, double, double)}.
         */
        public double elbowTargetDegrees = 0.0;

        /**
         * Wrist target angle after applying
         * {@link ArmIK2D#setOutputDirections(double, double, double)} and
         * {@link ArmIK2D#setOutputOffsetsDegrees(double, double, double)}.
         */
        public double wristTargetDegrees = 0.0;

        /**
         * Linear distance by which the requested wrist center was outside the reachable range. This is
         * zero for reachable targets.
         */
        public double reachError = 0.0;
    }

    private static class AngleLimit {
        private boolean enabled = false;
        private boolean largeArc = false;
        private double firstLimit = -180.0;
        private double secondLimit = 180.0;
        private double arcStart = -180.0;
        private double arcLength = 360.0;

        private void set(double firstLimitDegrees, double secondLimitDegrees, boolean useLargeArc) {
            enabled = true;
            largeArc = useLargeArc;
            firstLimit = normalizeAngle(firstLimitDegrees);
            secondLimit = normalizeAngle(secondLimitDegrees);

            double forwardDistance = positiveAngleDistance(firstLimit, secondLimit);
            boolean firstToSecondIsSmallArc = forwardDistance <= 180.0;
            double smallArcStart = firstToSecondIsSmallArc ? firstLimit : secondLimit;
            double smallArcLength = firstToSecondIsSmallArc ? forwardDistance : 360.0 - forwardDistance;

            if (largeArc) {
                arcStart = normalizeAngle(smallArcStart + smallArcLength);
                arcLength = 360.0 - smallArcLength;
            } else {
                arcStart = smallArcStart;
                arcLength = smallArcLength;
            }
        }

        private void disable() {
            enabled = false;
        }

        private boolean contains(double angleDegrees) {
            if (!enabled || arcLength >= 360.0 - EPSILON) {
                return true;
            }
            return positiveAngleDistance(arcStart, angleDegrees) <= arcLength + EPSILON;
        }
    }

    private double shoulderLength;
    private double elbowLength;
    private double wristLength;

    private double shoulderOffsetDegrees = 0.0;
    private double elbowOffsetDegrees = 0.0;
    private double wristOffsetDegrees = 0.0;
    private double shoulderDirection = 1.0;
    private double elbowDirection = 1.0;
    private double wristDirection = 1.0;

    private final AngleLimit shoulderLimit = new AngleLimit();
    private final AngleLimit elbowLimit = new AngleLimit();
    private final AngleLimit wristLimit = new AngleLimit();

    /**
     * Creates a planar 3-DOF IK solver.
     *
     * <p>The three lengths describe the physical arm geometry. They do not need to use any specific
     * unit, but all target coordinates passed into the solver must use the same unit.</p>
     *
     * @param shoulderLength Length from the shoulder pivot to the elbow pivot. Must be positive.
     * @param elbowLength Length from the elbow pivot to the wrist pivot. Must be positive.
     * @param wristLength Length from the wrist pivot to the end effector/tool center. Must be
     *     non-negative. Use {@code 0.0} if the wrist joint is also the target point.
     */
    public ArmIK2D(double shoulderLength, double elbowLength, double wristLength) {
        setLinkLengths(shoulderLength, elbowLength, wristLength);
    }

    /**
     * Creates a planar two-link arm solver with the end effector at the end of the elbow link.
     *
     * @param shoulderLength Length from the shoulder pivot to the elbow pivot. Must be positive.
     * @param elbowLength Length from the elbow pivot to the end effector. Must be positive.
     */
    public ArmIK2D(double shoulderLength, double elbowLength) {
        this(shoulderLength, elbowLength, 0.0);
    }

    /**
     * Updates the arm geometry.
     *
     * <p>This is useful if you prototype with different intake/tool lengths. The solver does not
     * validate lengths immediately; invalid lengths are reported during solve as
     * {@link Status#INVALID_LENGTHS}.</p>
     *
     * @param shoulderLength Length from shoulder pivot to elbow pivot. Must be positive to solve.
     * @param elbowLength Length from elbow pivot to wrist pivot. Must be positive to solve.
     * @param wristLength Length from wrist pivot to end effector. Must be finite and non-negative.
     */
    public void setLinkLengths(double shoulderLength, double elbowLength, double wristLength) {
        this.shoulderLength = shoulderLength;
        this.elbowLength = elbowLength;
        this.wristLength = wristLength;
    }

    /**
     * Sets fixed output angle offsets for all joints.
     *
     * <p>The raw IK angles are mathematical angles. Your encoder zero positions usually are not.
     * Offsets convert from the math frame into the target angles your {@link ClosedLoopDC} or servo
     * code expects. Output conversion is:</p>
     *
     * <pre>
     * outputAngle = normalize((mathAngle * direction) + offset)
     * </pre>
     *
     * @param shoulderOffsetDegrees Offset added to the shoulder output angle.
     * @param elbowOffsetDegrees Offset added to the elbow output angle.
     * @param wristOffsetDegrees Offset added to the wrist output angle.
     */
    public void setOutputOffsetsDegrees(
            double shoulderOffsetDegrees,
            double elbowOffsetDegrees,
            double wristOffsetDegrees) {
        this.shoulderOffsetDegrees = shoulderOffsetDegrees;
        this.elbowOffsetDegrees = elbowOffsetDegrees;
        this.wristOffsetDegrees = wristOffsetDegrees;
    }

    /**
     * Sets output direction signs for all joints.
     *
     * <p>Use {@code 1.0} when a joint's controller angle increases in the same direction as the
     * solver's math angle. Use {@code -1.0} when it increases in the opposite direction. A value of
     * {@code 0.0} is treated as {@code 1.0} so a bad dashboard value does not silence a joint.</p>
     *
     * @param shoulderDirection Shoulder output sign. Usually {@code 1.0} or {@code -1.0}.
     * @param elbowDirection Elbow output sign. Usually {@code 1.0} or {@code -1.0}.
     * @param wristDirection Wrist output sign. Usually {@code 1.0} or {@code -1.0}.
     */
    public void setOutputDirections(double shoulderDirection, double elbowDirection, double wristDirection) {
        this.shoulderDirection = signOrOne(shoulderDirection);
        this.elbowDirection = signOrOne(elbowDirection);
        this.wristDirection = signOrOne(wristDirection);
    }

    /**
     * Solves IK and returns a new result object.
     *
     * <p>This convenience method allocates a new {@link Result}. It is fine for setup, debugging, or
     * low-rate code. In a high-frequency FTC loop, prefer
     * {@link #solveInto(Result, double, double, double, boolean, boolean)} and reuse the same result
     * object.</p>
     *
     * <p>This method enables unreachable-target clamping by default. If the requested wrist center is
     * outside the arm's reachable range, the solver returns the closest reachable target and sets
     * {@link Result#targetClamped} plus {@link Status#TARGET_CLAMPED_TO_REACHABLE_RANGE}.</p>
     *
     * @param x Target end-effector x coordinate measured from the shoulder pivot.
     * @param y Target end-effector y coordinate measured from the shoulder pivot.
     * @param endEffectorDegrees Desired end-effector heading in degrees.
     * @param elbowUp True to use the elbow-up solution, false to use the elbow-down solution.
     * @return New result containing status, raw math angles, converted output angles, and diagnostics.
     */
    public Result solve(double x, double y, double endEffectorDegrees, boolean elbowUp) {
        Result result = new Result();
        solveInto(result, x, y, endEffectorDegrees, elbowUp, true);
        return result;
    }

    /**
     * Solves a two-link arm target and clamps unreachable targets to the reachable annulus.
     *
     * @param x Target end-effector x coordinate measured from the shoulder pivot.
     * @param y Target end-effector y coordinate measured from the shoulder pivot.
     * @param elbowUp True to use the elbow-up solution, false to use elbow-down.
     * @return New result containing the joint targets and solve diagnostics.
     */
    public Result solve(double x, double y, boolean elbowUp) {
        return solve(x, y, 0.0, elbowUp);
    }

    /**
     * Solves a two-link arm target into a reusable result object.
     *
     * @param result Reusable result object to overwrite.
     * @param x Target end-effector x coordinate measured from the shoulder pivot.
     * @param y Target end-effector y coordinate measured from the shoulder pivot.
     * @param elbowUp True to use the elbow-up solution, false to use elbow-down.
     * @param clampUnreachableTargets True to clamp unreachable targets, false to reject them.
     */
    public void solveInto(
            Result result,
            double x,
            double y,
            boolean elbowUp,
            boolean clampUnreachableTargets) {
        solveInto(result, x, y, 0.0, elbowUp, clampUnreachableTargets);
    }

    /**
     * Solves IK into an existing result object.
     *
     * <p>This is the preferred method for robot loop code because it avoids allocation. The solver
     * always overwrites every public field in {@code result}, so old solve data will not leak into the
     * next cycle.</p>
     *
     * <p>When {@code clampUnreachableTargets} is true, unreachable wrist centers are projected onto
     * the nearest reachable radius along the same ray from the shoulder. This is usually the safest
     * behavior for teleop/intake tracking because the arm still moves toward the nearest possible
     * point. When false, unreachable targets return {@link Status#UNREACHABLE} and no target angles
     * should be used.</p>
     *
     * <p>Joint limits are checked after output offsets and directions are applied. That means limits
     * should be written in the same angle frame you send to your motors/servos, not the raw math
     * frame.</p>
     *
     * @param result Reusable result object to overwrite. Must not be {@code null}.
     * @param x Target end-effector x coordinate measured from the shoulder pivot.
     * @param y Target end-effector y coordinate measured from the shoulder pivot.
     * @param endEffectorDegrees Desired final end-effector heading in degrees.
     * @param elbowUp True for the elbow-up branch of the two-link solution, false for elbow-down.
     * @param clampUnreachableTargets True to clamp unreachable wrist centers to the nearest reachable
     *     point, false to reject unreachable targets.
     */
    public void solveInto(
            Result result,
            double x,
            double y,
            double endEffectorDegrees,
            boolean elbowUp,
            boolean clampUnreachableTargets) {
        resetResult(result, x, y, endEffectorDegrees);

        if (!hasValidLengths()) {
            result.status = Status.INVALID_LENGTHS;
            return;
        }
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(endEffectorDegrees)) {
            result.status = Status.INVALID_TARGET;
            return;
        }

        double endEffectorRadians = endEffectorDegrees * DEGREES_TO_RADIANS;
        double wristX = x - wristLength * Math.cos(endEffectorRadians);
        double wristY = y - wristLength * Math.sin(endEffectorRadians);
        double wristDistance = Math.hypot(wristX, wristY);
        double minReach = Math.abs(shoulderLength - elbowLength);
        double maxReach = shoulderLength + elbowLength;

        result.wristCenterX = wristX;
        result.wristCenterY = wristY;
        result.reachable = wristDistance >= minReach - EPSILON && wristDistance <= maxReach + EPSILON;

        if (!result.reachable) {
            result.reachError = wristDistance > maxReach ? wristDistance - maxReach : minReach - wristDistance;
            if (!clampUnreachableTargets) {
                result.status = Status.UNREACHABLE;
                return;
            }

            double clampedDistance = Math.max(minReach, Math.min(maxReach, wristDistance));
            double scale = wristDistance > EPSILON ? clampedDistance / wristDistance : 1.0;
            wristX = wristDistance > EPSILON ? wristX * scale : clampedDistance;
            wristY = wristDistance > EPSILON ? wristY * scale : 0.0;
            wristDistance = clampedDistance;
            result.wristCenterX = wristX;
            result.wristCenterY = wristY;
            result.targetClamped = true;
        }

        double wristDistanceSquared = (wristDistance * wristDistance);
        double cosElbow =
                (wristDistanceSquared - (shoulderLength * shoulderLength) - (elbowLength * elbowLength))
                        / (2.0 * shoulderLength * elbowLength);
        cosElbow = clamp(cosElbow, -1.0, 1.0);

        double elbowRadians = Math.acos(cosElbow);
        if (elbowUp) {
            elbowRadians = -elbowRadians;
        }

        double shoulderRadians =
                Math.atan2(wristY, wristX)
                        - Math.atan2(
                                elbowLength * Math.sin(elbowRadians),
                                shoulderLength + elbowLength * Math.cos(elbowRadians));
        double wristRadians = endEffectorRadians - shoulderRadians - elbowRadians;

        result.shoulderDegrees = normalizeAngle(shoulderRadians * RADIANS_TO_DEGREES);
        result.elbowDegrees = normalizeAngle(elbowRadians * RADIANS_TO_DEGREES);
        result.wristDegrees = normalizeAngle(wristRadians * RADIANS_TO_DEGREES);

        result.shoulderTargetDegrees = toOutputAngle(result.shoulderDegrees, shoulderDirection, shoulderOffsetDegrees);
        result.elbowTargetDegrees = toOutputAngle(result.elbowDegrees, elbowDirection, elbowOffsetDegrees);
        result.wristTargetDegrees = toOutputAngle(result.wristDegrees, wristDirection, wristOffsetDegrees);

        result.withinJointLimits =
                shoulderLimit.contains(result.shoulderTargetDegrees)
                        && elbowLimit.contains(result.elbowTargetDegrees)
                        && wristLimit.contains(result.wristTargetDegrees);

        result.solvedX =
                shoulderLength * Math.cos(shoulderRadians)
                        + elbowLength * Math.cos(shoulderRadians + elbowRadians)
                        + wristLength * Math.cos(endEffectorRadians);
        result.solvedY =
                shoulderLength * Math.sin(shoulderRadians)
                        + elbowLength * Math.sin(shoulderRadians + elbowRadians)
                        + wristLength * Math.sin(endEffectorRadians);

        if (!result.withinJointLimits) {
            result.status = Status.OUTSIDE_JOINT_LIMITS;
            return;
        }

        result.valid = true;
        result.status = result.targetClamped ? Status.TARGET_CLAMPED_TO_REACHABLE_RANGE : Status.VALID;
    }

    /**
     * Enables a shoulder output limit using the smaller arc between two endpoint angles.
     *
     * @param firstLimitDegrees First shoulder limit endpoint in output degrees.
     * @param secondLimitDegrees Second shoulder limit endpoint in output degrees.
     */
    public void enableSmallArcShoulderLimit(double firstLimitDegrees, double secondLimitDegrees) {
        shoulderLimit.set(firstLimitDegrees, secondLimitDegrees, false);
    }

    /**
     * Enables a shoulder output limit using the larger arc between two endpoint angles.
     *
     * @param firstLimitDegrees First shoulder limit endpoint in output degrees.
     * @param secondLimitDegrees Second shoulder limit endpoint in output degrees.
     */
    public void enableLargeArcShoulderLimit(double firstLimitDegrees, double secondLimitDegrees) {
        shoulderLimit.set(firstLimitDegrees, secondLimitDegrees, true);
    }

    /**
     * Enables an elbow output limit using the smaller arc between two endpoint angles.
     *
     * @param firstLimitDegrees First elbow limit endpoint in output degrees.
     * @param secondLimitDegrees Second elbow limit endpoint in output degrees.
     */
    public void enableSmallArcElbowLimit(double firstLimitDegrees, double secondLimitDegrees) {
        elbowLimit.set(firstLimitDegrees, secondLimitDegrees, false);
    }

    /**
     * Enables an elbow output limit using the larger arc between two endpoint angles.
     *
     * @param firstLimitDegrees First elbow limit endpoint in output degrees.
     * @param secondLimitDegrees Second elbow limit endpoint in output degrees.
     */
    public void enableLargeArcElbowLimit(double firstLimitDegrees, double secondLimitDegrees) {
        elbowLimit.set(firstLimitDegrees, secondLimitDegrees, true);
    }

    /**
     * Enables a wrist output limit using the smaller arc between two endpoint angles.
     *
     * @param firstLimitDegrees First wrist limit endpoint in output degrees.
     * @param secondLimitDegrees Second wrist limit endpoint in output degrees.
     */
    public void enableSmallArcWristLimit(double firstLimitDegrees, double secondLimitDegrees) {
        wristLimit.set(firstLimitDegrees, secondLimitDegrees, false);
    }

    /**
     * Enables a wrist output limit using the larger arc between two endpoint angles.
     *
     * @param firstLimitDegrees First wrist limit endpoint in output degrees.
     * @param secondLimitDegrees Second wrist limit endpoint in output degrees.
     */
    public void enableLargeArcWristLimit(double firstLimitDegrees, double secondLimitDegrees) {
        wristLimit.set(firstLimitDegrees, secondLimitDegrees, true);
    }

    /**
     * Disables the shoulder output limit.
     */
    public void disableShoulderLimit() {
        shoulderLimit.disable();
    }

    /**
     * Disables the elbow output limit.
     */
    public void disableElbowLimit() {
        elbowLimit.disable();
    }

    /**
     * Disables the wrist output limit.
     */
    public void disableWristLimit() {
        wristLimit.disable();
    }

    /**
     * Disables shoulder, elbow, and wrist output limits.
     */
    public void disableJointLimits() {
        disableShoulderLimit();
        disableElbowLimit();
        disableWristLimit();
    }

    /**
     * @return Configured shoulder link length.
     */
    public double getShoulderLength() {
        return shoulderLength;
    }

    /**
     * @return Configured elbow link length.
     */
    public double getElbowLength() {
        return elbowLength;
    }

    /**
     * @return Configured wrist/tool length.
     */
    public double getWristLength() {
        return wristLength;
    }

    private boolean hasValidLengths() {
        return isPositiveFinite(shoulderLength)
                && isPositiveFinite(elbowLength)
                && Double.isFinite(wristLength)
                && wristLength >= 0.0;
    }

    private static void resetResult(Result result, double x, double y, double endEffectorDegrees) {
        result.status = Status.INVALID_TARGET;
        result.valid = false;
        result.reachable = false;
        result.targetClamped = false;
        result.withinJointLimits = true;
        result.requestedX = x;
        result.requestedY = y;
        result.requestedEndEffectorDegrees = normalizeAngle(endEffectorDegrees);
        result.solvedX = 0.0;
        result.solvedY = 0.0;
        result.wristCenterX = 0.0;
        result.wristCenterY = 0.0;
        result.shoulderDegrees = 0.0;
        result.elbowDegrees = 0.0;
        result.wristDegrees = 0.0;
        result.shoulderTargetDegrees = 0.0;
        result.elbowTargetDegrees = 0.0;
        result.wristTargetDegrees = 0.0;
        result.reachError = 0.0;
    }

    private static boolean isPositiveFinite(double value) {
        return Double.isFinite(value) && value > EPSILON;
    }

    private static double signOrOne(double value) {
        double sign = Math.signum(value);
        return sign == 0.0 ? 1.0 : sign;
    }

    private static double toOutputAngle(double mathDegrees, double direction, double offsetDegrees) {
        return normalizeAngle((mathDegrees * direction) + offsetDegrees);
    }

    private static double normalizeAngle(double angle) {
        if (!Double.isFinite(angle)) {
            return 0.0;
        }
        angle = (angle + 180.0) % 360.0;
        if (angle < 0.0) {
            angle += 360.0;
        }
        angle -= 180.0;
        return angle <= -180.0 ? 180.0 : angle;
    }

    private static double positiveAngleDistance(double from, double to) {
        double distance = normalizeAngle(to) - normalizeAngle(from);
        distance %= 360.0;
        if (distance < 0.0) {
            distance += 360.0;
        }
        return distance;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
