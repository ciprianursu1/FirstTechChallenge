# ArmIK2D Usage

`ArmIK2D` is a closed-form inverse kinematics module for planar 2-link and 3-DOF FTC arms.

It is meant for an arm shaped like:

```text
shoulder pivot -> shoulder link -> elbow pivot -> elbow link -> wrist pivot -> wrist/tool
```

It solves:

```text
target x
target y
target end-effector angle
```

and outputs:

```text
shoulder target angle
elbow target angle
wrist target angle
```

The solver does not use iteration, so it is efficient enough to run every loop on a Control Hub.

## Coordinate System

All target positions are measured from the shoulder pivot.

```text
x = horizontal distance from shoulder pivot
y = vertical distance from shoulder pivot
angle = desired final tool angle
```

Positive angles follow normal math convention:

```text
0 deg   = pointing forward along +x
90 deg  = pointing upward along +y
-90 deg = pointing downward
180 deg = pointing backward
```

All angles returned by the module are normalized to:

```text
(-180, 180]
```

That means `180` is possible, but `-180` is converted to `180`.

## Link Lengths

Create the solver with your physical arm lengths:

```java
ArmIK2D ik = new ArmIK2D(14.0, 12.0, 4.0);
```

The values can be inches, centimeters, or any other linear unit. The only rule is that `x`, `y`, and all link lengths must use the same unit.

```text
shoulderLength = shoulder pivot to elbow pivot
elbowLength    = elbow pivot to wrist pivot
wristLength    = wrist pivot to intake/tool center
```

If your wrist pivot is the actual target point, use:

```java
ArmIK2D ik = new ArmIK2D(14.0, 12.0, 0.0);
```

For a two-link arm with no wrist joint, use the two-argument constructor:

```java
ArmIK2D ik = new ArmIK2D(0.360, 0.340);
```

The two-link solve overload does not require an end-effector heading:

```java
ik.solveInto(ikResult, targetX, targetY, true, false);
```

Here, `true` selects elbow-up and `false` rejects unreachable targets. Rejecting an unreachable
target is useful for Cartesian D-pad control because it prevents one axis from moving unexpectedly
when the requested movement is physically impossible.

## Basic Loop Usage

Reuse a `Result` object to avoid garbage collection during the loop:

```java
private final ArmIK2D ik = new ArmIK2D(14.0, 12.0, 4.0);
private final ArmIK2D.Result ikResult = new ArmIK2D.Result();

public void loop() {
    double targetX = 18.0;
    double targetY = 6.0;
    double targetHeadingDegrees = 0.0;

    ik.solveInto(
            ikResult,
            targetX,
            targetY,
            targetHeadingDegrees,
            false,
            true);

    if (ikResult.valid) {
        shoulder.update(ikResult.shoulderTargetDegrees);
        elbow.update(ikResult.elbowTargetDegrees);
        wrist.update(ikResult.wristTargetDegrees);
    }
}
```

The arguments are:

```text
result                    reusable output object
x                         target end-effector x
y                         target end-effector y
endEffectorDegrees        final intake/tool heading
elbowUp                   solution branch
clampUnreachableTargets   whether unreachable targets become nearest reachable targets
```

## Elbow Up vs Elbow Down

Most 2-link arms can reach the same point in two ways:

```text
elbowUp = true
elbowUp = false
```

Try both on the robot with low max power and pick the one that matches your mechanism.

For intakes, you usually want to keep this fixed during a mode. Flipping it live can cause the arm to choose a very different posture for the same target.

## Using With ClosedLoopDC

For motor-driven joints, configure each `ClosedLoopDC` in angle mode:

```java
shoulder.setAngleMode(true);
elbow.setAngleMode(true);
wrist.setAngleMode(true);
```

Then pass the IK output targets:

```java
if (ikResult.valid) {
    shoulder.update(ikResult.shoulderTargetDegrees);
    elbow.update(ikResult.elbowTargetDegrees);
    wrist.update(ikResult.wristTargetDegrees);
}
```

If a joint is servo-driven, convert degrees to servo position yourself:

```java
double servoPosition = (ikResult.wristTargetDegrees - minDegrees) / (maxDegrees - minDegrees);
servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));
wristServo.setPosition(servoPosition);
```

## Output Offsets

The raw IK angles are mathematical angles. Your encoders probably do not read `0` when the link points along +x.

Use output offsets to convert math angles into your controller's angle frame:

```java
ik.setOutputOffsetsDegrees(
        shoulderOffsetDegrees,
        elbowOffsetDegrees,
        wristOffsetDegrees);
```

The conversion is:

```text
target = normalize((mathAngle * direction) + offset)
```

Example:

```java
ik.setOutputOffsetsDegrees(12.0, -5.0, 90.0);
```

## Output Directions

If a joint moves backward when commanded, flip its output direction:

```java
ik.setOutputDirections(1.0, -1.0, 1.0);
```

Use:

```text
1.0  = controller angle increases in the same direction as the IK math angle
-1.0 = controller angle increases opposite the IK math angle
```

Do this before tuning PID seriously. Wrong signs make the controller look broken.

## Joint Limits

Joint limits are checked after output offsets and directions are applied.

That means the limit values should be written in the same angle frame you send to your motors.

Small arc limit:

```java
ik.enableSmallArcShoulderLimit(-90.0, 90.0);
```

This allows the shorter path between the two endpoints.

Large arc limit:

```java
ik.enableLargeArcShoulderLimit(-90.0, 90.0);
```

This allows the opposite side of the circle, through `180`.

Available limit methods:

```java
ik.enableSmallArcShoulderLimit(a, b);
ik.enableLargeArcShoulderLimit(a, b);
ik.enableSmallArcElbowLimit(a, b);
ik.enableLargeArcElbowLimit(a, b);
ik.enableSmallArcWristLimit(a, b);
ik.enableLargeArcWristLimit(a, b);

ik.disableShoulderLimit();
ik.disableElbowLimit();
ik.disableWristLimit();
ik.disableJointLimits();
```

If a solution violates limits:

```java
ikResult.valid == false
ikResult.status == ArmIK2D.Status.OUTSIDE_JOINT_LIMITS
```

## Reachability

The shoulder and elbow solve to the wrist center, not directly to the end-effector point.

The wrist center is:

```text
wristX = targetX - wristLength * cos(targetHeading)
wristY = targetY - wristLength * sin(targetHeading)
```

Reach range:

```text
minimum reach = abs(shoulderLength - elbowLength)
maximum reach = shoulderLength + elbowLength
```

If `clampUnreachableTargets` is true, an unreachable target is moved to the nearest reachable point:

```java
ik.solveInto(result, x, y, angle, false, true);
```

Then:

```text
result.valid = true
result.targetClamped = true
result.status = TARGET_CLAMPED_TO_REACHABLE_RANGE
```

If `clampUnreachableTargets` is false:

```java
ik.solveInto(result, x, y, angle, false, false);
```

Then unreachable targets are rejected:

```text
result.valid = false
result.status = UNREACHABLE
```

For autonomous scoring positions, rejecting unreachable targets is usually better. For teleop tracking or vision targeting, clamping is usually smoother.

## Result Fields

Important fields:

```text
status                  why the solve succeeded or failed
valid                   true if targets are safe to command
reachable               true if the original target was reachable
targetClamped           true if the target was moved into range
withinJointLimits       true if enabled joint limits accepted the target
shoulderTargetDegrees   motor/controller-ready shoulder target
elbowTargetDegrees      motor/controller-ready elbow target
wristTargetDegrees      motor/controller-ready wrist target
reachError              distance outside reachable range
```

Debug fields:

```text
requestedX
requestedY
requestedEndEffectorDegrees
solvedX
solvedY
wristCenterX
wristCenterY
shoulderDegrees
elbowDegrees
wristDegrees
```

## Status Values

```text
VALID
```

The target was reachable, inside joint limits, and solved normally.

```text
TARGET_CLAMPED_TO_REACHABLE_RANGE
```

The target was unreachable, but clamping was enabled and the closest reachable target was solved.

```text
INVALID_LENGTHS
```

The arm geometry is invalid. Shoulder and elbow lengths must be positive. Wrist length must be non-negative.

```text
INVALID_TARGET
```

The requested `x`, `y`, or angle was `NaN` or infinite.

```text
UNREACHABLE
```

The requested target is outside the reachable range and clamping was disabled.

```text
OUTSIDE_JOINT_LIMITS
```

The pose is geometrically possible, but at least one joint target violates configured limits.

## Calibration Workflow

1. Measure link lengths from pivot center to pivot center.
2. Set `wristLength` from wrist pivot to the intake/contact point you care about.
3. Put the robot in a known physical pose.
4. Compute or estimate what the math angles should be in that pose.
5. Read your encoder/controller angles.
6. Set output directions first.
7. Set output offsets second.
8. Enable conservative joint limits.
9. Test with low `ClosedLoopDC.maxPower`.
10. Only then tune faster profiles.

## Common Problems

If the arm moves mirrored:

```text
Flip one or more output directions.
```

If the shoulder looks correct but the elbow is folded the wrong way:

```text
Switch elbowUp true/false.
```

If the wrist angle looks consistently offset:

```text
Adjust wristOffsetDegrees.
```

If `valid` is false and status is `OUTSIDE_JOINT_LIMITS`:

```text
The IK math found a pose, but your configured joint limits rejected it.
```

If the target is reachable without the wrist, but unreachable with the wrist:

```text
The requested end-effector angle is moving the wrist center outside the shoulder/elbow workspace.
```

If motion is unstable:

```text
IK is only target generation. Stability still depends on your joint controllers, gear ratio, gravity feedforward, velocity damping, and motion profiles.
```

## Performance Notes

Use this in loop code:

```java
private final ArmIK2D.Result ikResult = new ArmIK2D.Result();
```

Then:

```java
ik.solveInto(ikResult, x, y, angle, elbowUp, true);
```

Avoid this in tight loops if you care about allocation:

```java
ArmIK2D.Result result = ik.solve(x, y, angle, elbowUp);
```

The math cost is fixed per solve. There is no loop, search, optimizer, matrix solve, or numerical iteration.
