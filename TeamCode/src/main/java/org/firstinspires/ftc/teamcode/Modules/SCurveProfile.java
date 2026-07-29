package org.firstinspires.ftc.teamcode.Modules;

/**
 * 7-Segment Jerk-Limited S-Curve Motion Profile Generator.
 * Calculates kinematic targets (position, velocity, acceleration) for smooth physical movement.
 */
public class SCurveProfile {
    private static final double EPSILON = 1e-9;

    private double maxVel;
    private double maxAccel;
    private double maxJerk;

    private double t1, t2, t3, t4, t5, t6, t7;
    private double totalTime;

    private double actualMaxAccel;
    private double actualMaxVel;
    private double distance;
    private int direction = 1;

    /**
     * Holds state data for a specific timestamp along the profile.
     */
    public static class ProfileState {
        public double position;
        public double velocity;
        public double acceleration;

        public ProfileState(double position, double velocity, double acceleration) {
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
        }
    }

    /**
     * Constructs and generates a 7-segment S-curve profile.
     *
     * @param maxVel   Maximum velocity limit (units/sec)
     * @param maxAccel Maximum acceleration limit (units/sec^2)
     * @param maxJerk  Maximum jerk limit (units/sec^3)
     * @param distance Total travel distance (units)
     */
    public void generate(double maxVel, double maxAccel, double maxJerk, double distance) {
        if (!isUsable(maxVel) || !isUsable(maxAccel) || !isUsable(maxJerk) || !Double.isFinite(distance)) {
            resetProfile();
            return;
        }

        this.direction = distance >= 0 ? 1 : -1;
        this.distance = Math.abs(distance);
        this.maxVel = Math.abs(maxVel);
        this.maxAccel = Math.abs(maxAccel);
        this.maxJerk = Math.abs(maxJerk);

        if (this.distance < EPSILON) {
            resetProfile();
            return;
        }

        double maxReachableVel = this.maxVel;
        double fullAccelDistance =
                calculateAccelerationDistance(maxReachableVel, this.maxAccel, this.maxJerk);

        if (this.distance >= 2.0 * fullAccelDistance) {
            actualMaxVel = maxReachableVel;
            configureAccelTimesForVelocity(actualMaxVel);
            t4 = (this.distance - (2.0 * fullAccelDistance)) / actualMaxVel;
        } else {
            actualMaxVel = calculateVelocityForAccelerationDistance(this.distance / 2.0, this.maxAccel, this.maxJerk);
            configureAccelTimesForVelocity(actualMaxVel);
            t4 = 0.0;
        }

        totalTime = t1 + t2 + t3 + t4 + t5 + t6 + t7;
    }

    /**
     * Computes a conservative jerk-limited stopping distance from current speed to zero speed.
     */
    public static double calculateStoppingDistance(double velocity, double maxAccel, double maxJerk) {
        return calculateAccelerationDistance(velocity, maxAccel, maxJerk);
    }

    /**
     * Computes the distance needed to accelerate from zero to {@code velocity}
     * with zero start and end acceleration.
     */
    public static double calculateAccelerationDistance(double velocity, double maxAccel, double maxJerk) {
        if (!isUsable(maxAccel) || !isUsable(maxJerk) || !Double.isFinite(velocity)) {
            return 0.0;
        }

        double v = Math.abs(velocity);
        if (v < EPSILON) {
            return 0.0;
        }

        double accel = Math.abs(maxAccel);
        double jerk = Math.abs(maxJerk);
        double velocityLostDuringJerk = (accel * accel) / jerk;

        if (v <= velocityLostDuringJerk) {
            double jerkTime = Math.sqrt(v / jerk);
            return v * jerkTime;
        }

        double jerkTime = accel / jerk;
        double constantAccelTime = (v - velocityLostDuringJerk) / accel;
        return 0.5 * v * ((2.0 * jerkTime) + constantAccelTime);
    }

    /**
     * Computes the time needed to accelerate from zero to {@code velocity}
     * with zero start and end acceleration.
     */
    public static double calculateAccelerationTime(double velocity, double maxAccel, double maxJerk) {
        if (!isUsable(maxAccel) || !isUsable(maxJerk) || !Double.isFinite(velocity)) {
            return 0.0;
        }

        double v = Math.abs(velocity);
        if (v < EPSILON) {
            return 0.0;
        }

        double accel = Math.abs(maxAccel);
        double jerk = Math.abs(maxJerk);
        double velocityLostDuringJerk = (accel * accel) / jerk;

        if (v <= velocityLostDuringJerk) {
            return 2.0 * Math.sqrt(v / jerk);
        }

        return (2.0 * accel / jerk) + ((v - velocityLostDuringJerk) / accel);
    }

    /**
     * Calculates the target position, velocity, and acceleration at time {@code t}.
     *
     * @param t Query timestamp in seconds
     * @return Target {@link ProfileState} at time {@code t}
     */
    public ProfileState calculate(double t) {
        if (!Double.isFinite(t) || t <= 0 || totalTime <= 0) {
            return new ProfileState(0, 0, 0);
        }
        if (t >= totalTime) {
            return new ProfileState(distance * direction, 0, 0);
        }

        double pos;
        double vel;
        double accel;

        // Phase 1: Accel Ramp Up
        if (t <= t1) {
            accel = maxJerk * t;
            vel = 0.5 * maxJerk * t * t;
            pos = (1.0 / 6.0) * maxJerk * t * t * t;
        }
        // Phase 2: Constant Accel
        else if (t <= t1 + t2) {
            double dt = t - t1;
            double v1 = 0.5 * maxJerk * t1 * t1;
            double p1 = (1.0 / 6.0) * maxJerk * t1 * t1 * t1;

            accel = actualMaxAccel;
            vel = v1 + actualMaxAccel * dt;
            pos = p1 + v1 * dt + 0.5 * actualMaxAccel * dt * dt;
        }
        // Phase 3: Accel Ramp Down
        else if (t <= t1 + t2 + t3) {
            double dt = t - (t1 + t2);
            double v2 = 0.5 * maxJerk * t1 * t1 + actualMaxAccel * t2;
            double p2 = (1.0 / 6.0) * maxJerk * t1 * t1 * t1 + (0.5 * maxJerk * t1 * t1) * t2 + 0.5 * actualMaxAccel * t2 * t2;

            accel = actualMaxAccel - maxJerk * dt;
            vel = v2 + actualMaxAccel * dt - 0.5 * maxJerk * dt * dt;
            pos = p2 + v2 * dt + 0.5 * actualMaxAccel * dt * dt - (1.0 / 6.0) * maxJerk * dt * dt * dt;
        }
        // Phase 4: Cruise
        else if (t <= t1 + t2 + t3 + t4) {
            double dt = t - (t1 + t2 + t3);
            double p3 = calculatePhase3EndPos();

            accel = 0;
            vel = actualMaxVel;
            pos = p3 + actualMaxVel * dt;
        }
        // Deceleration phases (5, 6, 7 mirrored)
        else {
            double tDecel = totalTime - t;
            ProfileState mirrored = calculate(tDecel);

            accel = -mirrored.acceleration;
            vel = mirrored.velocity;
            pos = distance - mirrored.position;
        }

        return new ProfileState(pos * direction, vel * direction, accel * direction);
    }

    private double calculatePhase3EndPos() {
        double p1 = (1.0 / 6.0) * maxJerk * t1 * t1 * t1;
        double v1 = 0.5 * maxJerk * t1 * t1;
        double p2 = p1 + v1 * t2 + 0.5 * actualMaxAccel * t2 * t2;
        double v2 = v1 + actualMaxAccel * t2;
        return p2 + v2 * t3 + 0.5 * actualMaxAccel * t3 * t3 - (1.0 / 6.0) * maxJerk * t3 * t3 * t3;
    }

    public double getTotalTime() {
        return totalTime;
    }

    public double getDistance() {
        return distance * direction;
    }

    private void configureAccelTimesForVelocity(double velocity) {
        double v = Math.max(0.0, Math.min(Math.abs(velocity), maxVel));
        double velocityLostDuringJerk = (maxAccel * maxAccel) / maxJerk;

        if (v <= velocityLostDuringJerk) {
            t1 = Math.sqrt(v / maxJerk);
            t2 = 0.0;
            t3 = t1;
            actualMaxAccel = maxJerk * t1;
        } else {
            t1 = maxAccel / maxJerk;
            t2 = (v - velocityLostDuringJerk) / maxAccel;
            t3 = t1;
            actualMaxAccel = maxAccel;
        }

        actualMaxVel = v;
        t5 = t1;
        t6 = t2;
        t7 = t3;
    }

    private static double calculateVelocityForAccelerationDistance(double distance, double maxAccel, double maxJerk) {
        if (!isUsable(maxAccel) || !isUsable(maxJerk) || !Double.isFinite(distance) || distance <= 0) {
            return 0.0;
        }

        double accel = Math.abs(maxAccel);
        double jerk = Math.abs(maxJerk);
        double jerkOnlyLimitDistance = (accel * accel * accel) / (jerk * jerk);

        if (distance <= jerkOnlyLimitDistance) {
            return Math.cbrt(distance * distance * jerk);
        }

        double b = (accel * accel) / jerk;
        return 0.5 * (-b + Math.sqrt((b * b) + (8.0 * accel * distance)));
    }

    private static boolean isUsable(double value) {
        return Double.isFinite(value) && Math.abs(value) > EPSILON;
    }

    private void resetProfile() {
        t1 = t2 = t3 = t4 = t5 = t6 = t7 = 0.0;
        totalTime = 0.0;
        actualMaxAccel = 0.0;
        actualMaxVel = 0.0;
        distance = 0.0;
        direction = 1;
    }
}
