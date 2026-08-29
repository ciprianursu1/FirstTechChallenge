package org.firstinspires.ftc.teamcode.Modules;

import com.pedropathing.geometry.Pose;
import java.util.Arrays;

/**
 * High-performance circular fixed-size ring buffer designed for storing timestamped
 * robot pose history (e.g., from Pedro Pathing or Pinpoint Odometry).
 * <p>
 * Primary Purpose: Resolves computer vision pipeline latency (e.g., Limelight 3A or OpenCV).
 * By keeping a continuous rolling history of high-frequency odometry poses, vision updates
 * can be queried against the exact point in time a camera frame was captured rather than
 * the delayed moment it was received.
 * </p>
 * * @author FTC Team Code
 * @version 2.0
 */
public class PositionRingBuffer {

    /**
     * Immutable data container representing a single pose record tagged with an execution timestamp.
     */
    public static class PositionData {
        /** The 2D pose (X, Y, Heading) of the robot on the field. */
        public Pose pose;

        /** System timestamp in milliseconds (obtained via {@link System#currentTimeMillis()}). */
        public long timestamp;

        /**
         * Constructs a timestamped position record.
         *
         * @param pose      The {@link Pose} object representing robot field coordinates.
         * @param timestamp Epoch timestamp in milliseconds.
         */
        public PositionData(Pose pose, long timestamp) {
            this.pose = pose;
            this.timestamp = timestamp;
        }
    }

    /** Maximum capacity of the circular buffer. */
    private final int bufferSize;

    /** Internal index pointer tracking the next insertion slot in the array. */
    private int bufferIndex = 0;

    /** Internal array storage for circular pose history records. */
    private final PositionData[] buffer;

    /**
     * Initializes a circular pose buffer with a fixed history size.
     *
     * @param bufferSize The maximum number of historical entries to store.
     * (e.g., A size of 50 at 100Hz loop speeds maintains ~500ms of history).
     */
    public PositionRingBuffer(int bufferSize) {
        this.bufferSize = bufferSize;
        this.buffer = new PositionData[bufferSize];
        Arrays.fill(buffer, null);
    }

    /**
     * Inserts a new pose into the buffer, tagging it automatically with the current system time in milliseconds.
     * Overwrites the oldest recorded entry once capacity is reached.
     *
     * @param pose The current live {@link Pose} of the drivetrain or odometry estimator.
     */
    public void update(Pose pose) {
        buffer[bufferIndex] = new PositionData(pose, System.currentTimeMillis());
        bufferIndex = (bufferIndex + 1) % bufferSize;
    }

    /**
     * Inserts an explicit pre-constructed {@link PositionData} entry into the circular buffer.
     *
     * @param poseData The timestamped position record to store.
     */
    public void update(PositionData poseData) {
        buffer[bufferIndex] = poseData;
        bufferIndex = (bufferIndex + 1) % bufferSize;
    }

    /**
     * Queries the buffer and returns the single discrete historical pose whose timestamp
     * most closely matches the requested timestamp.
     *
     * @param timestampMs Target timestamp in milliseconds.
     * @return The closest {@link PositionData} entry recorded at that point in time, or {@code null} if buffer is empty.
     */
    public PositionData getClosestPose(long timestampMs) {
        PositionData closest = null;
        long minDiff = Long.MAX_VALUE;

        for (int i = 0; i < bufferSize; i++) {
            if (buffer[i] == null) continue;

            long diff = Math.abs(buffer[i].timestamp - timestampMs);
            if (diff < minDiff) {
                minDiff = diff;
                closest = buffer[i];
            }
        }
        return closest;
    }

    /**
     * Performs linear interpolation (LERP) between the two closest recorded pose entries surrounding
     * the target timestamp.
     * <p>
     * Recommended over {@link #getClosestPose(long)} when running loop rates under 100Hz to eliminate
     * quantization jitter in historical position reconstruction.
     * </p>
     *
     * @param timestampMs Target timestamp in milliseconds.
     * @return A smooth interpolated {@link Pose} calculated at the requested historical moment,
     * or {@code null} if insufficient history exists.
     */
    public Pose getInterpolatedPose(long timestampMs) {
        PositionData before = null;
        PositionData after = null;

        for (int i = 0; i < bufferSize; i++) {
            PositionData data = buffer[i];
            if (data == null) continue;

            if (data.timestamp <= timestampMs) {
                if (before == null || data.timestamp > before.timestamp) {
                    before = data;
                }
            }
            if (data.timestamp >= timestampMs) {
                if (after == null || data.timestamp < after.timestamp) {
                    after = data;
                }
            }
        }

        // Edge case handling: If out of bounds or missing surrounding entries
        if (before == null && after == null) return null;
        if (before == null) return after.pose;
        if (after == null) return before.pose;
        if (before.timestamp == after.timestamp) return before.pose;

        // Compute interpolation ratio alpha inside interval [0.0, 1.0]
        double alpha = (double) (timestampMs - before.timestamp) / (after.timestamp - before.timestamp);

        // Linear interpolation across spatial axes and rotation
        double x = before.pose.getX() + alpha * (after.pose.getX() - before.pose.getX());
        double y = before.pose.getY() + alpha * (after.pose.getY() - before.pose.getY());
        double heading = before.pose.getHeading() + alpha * (after.pose.getHeading() - before.pose.getHeading());

        return new Pose(x, y, heading);
    }
}
