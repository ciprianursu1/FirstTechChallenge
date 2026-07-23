package org.firstinspires.ftc.teamcode.Subsystems;

/**
 * 7-Segment Jerk-Limited S-Curve Motion Profile Generator.
 * Calculates kinematic targets (position, velocity, acceleration) for smooth physical movement.
 */
public class SCurveProfile {

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
        this.direction = distance >= 0 ? 1 : -1;
        this.distance = Math.abs(distance);
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxJerk = maxJerk;

        if (this.distance == 0) {
            t1 = t2 = t3 = t4 = t5 = t6 = t7 = totalTime = 0;
            return;
        }

        // Time required to reach peak acceleration
        double tj = maxAccel / maxJerk;

        // Check if max acceleration is reachable
        if (maxVel < tj * maxAccel) {
            tj = Math.sqrt(maxVel / maxJerk);
            actualMaxAccel = tj * maxJerk;
        } else {
            actualMaxAccel = maxAccel;
        }

        // Velocity reached after accel ramp up + ramp down
        double vAccelRamp = actualMaxAccel * tj;

        // Distance covered during acceleration phase (Ramp up + Constant Accel + Ramp down)
        // If profile can reach max velocity
        if (this.distance < 2 * vAccelRamp) {
            // Can't reach max accel or max vel
            tj = Math.cbrt(this.distance / (2.0 * maxJerk));
            actualMaxAccel = tj * maxJerk;
            actualMaxVel = actualMaxAccel * tj;

            t1 = tj;
            t2 = 0; // No constant acceleration phase
            t3 = tj;
            t4 = 0; // No cruising phase
            t5 = tj;
            t6 = 0;
            t7 = tj;
        } else {
            // Check if max velocity is reached
            double dAccelPhase = actualMaxAccel * (tj * tj) + (maxVel - vAccelRamp) * (actualMaxAccel / maxJerk + (maxVel - vAccelRamp) / actualMaxAccel);

            if (this.distance >= 2 * dAccelPhase) {
                actualMaxVel = maxVel;
                t1 = tj;
                t2 = (actualMaxVel - vAccelRamp) / actualMaxAccel;
                t3 = tj;

                double dAccel = actualMaxVel * (t1 + t2);
                double dCruise = this.distance - 2 * dAccel;
                t4 = dCruise / actualMaxVel;

                t5 = tj;
                t6 = t2;
                t7 = tj;
            } else {
                // Reaches max accel, but not max velocity
                t1 = tj;
                t3 = tj;
                double vPeak = Math.sqrt(actualMaxAccel * actualMaxAccel * tj * tj + actualMaxAccel * (this.distance - 2 * actualMaxAccel * tj * tj));
                actualMaxVel = vPeak;
                t2 = (vPeak - vAccelRamp) / actualMaxAccel;
                t4 = 0; // No cruise phase

                t5 = tj;
                t6 = t2;
                t7 = tj;
            }
        }

        totalTime = t1 + t2 + t3 + t4 + t5 + t6 + t7;
    }

    /**
     * Calculates the target position, velocity, and acceleration at time {@code t}.
     *
     * @param t Query timestamp in seconds
     * @return Target {@link ProfileState} at time {@code t}
     */
    public ProfileState calculate(double t) {
        if (t <= 0) return new ProfileState(0, 0, 0);
        if (t >= totalTime) return new ProfileState(distance * direction, 0, 0);

        double pos = 0;
        double vel = 0;
        double accel = 0;

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
}