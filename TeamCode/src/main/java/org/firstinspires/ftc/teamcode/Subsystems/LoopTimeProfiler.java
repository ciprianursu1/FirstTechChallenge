package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * LoopTimeProfiler measures control loop execution time, loop frequency (Hz),
 * and tracks peak latency spikes.
 * <p>
 * Features exponential moving average (EMA) filtering for smooth dashboard/telemetry readings,
 * as well as optional FTC Telemetry integration.
 */
public class LoopTimeProfiler {

    private final Telemetry telemetry;

    private long lastTimeNs = 0;
    private long loopTimeNs = 0;
    private long maxLoopTimeNs = 0;

    /** Exponential Moving Average smoothing factor (0.0 to 1.0). Lower = smoother. */
    private static final double ALPHA = 0.1;
    private double smoothedLoopTimeNs = 0;

    /**
     * Default constructor for headless profiling (e.g., Panels Dashboard, unit tests, blackbox logging).
     */
    public LoopTimeProfiler() {
        this(null);
    }

    /**
     * Constructor with automatic FTC Telemetry reporting.
     *
     * @param telemetry The FTC OpMode {@link Telemetry} object used for logging telemetry data.
     */
    public LoopTimeProfiler(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Initializes or resets the profiler timer.
     * <p>
     * Call this immediately before entering the main OpMode {@code while(opModeIsActive())} loop.
     */
    public void start() {
        lastTimeNs = System.nanoTime();
        maxLoopTimeNs = 0;
        smoothedLoopTimeNs = 0;
    }

    /**
     * Updates loop timing calculations.
     * <p>
     * Call this exactly once per control loop iteration (preferably at the end of the loop).
     * If a {@link Telemetry} object was provided, it automatically logs timing stats.
     */
    public void update() {
        long now = System.nanoTime();
        loopTimeNs = now - lastTimeNs;
        lastTimeNs = now;

        // Ignore invalid initial frames
        if (loopTimeNs <= 0) return;

        // Track peak latency spikes
        if (loopTimeNs > maxLoopTimeNs) {
            maxLoopTimeNs = loopTimeNs;
        }

        // Apply Exponential Moving Average (EMA) filter
        if (smoothedLoopTimeNs == 0) {
            smoothedLoopTimeNs = loopTimeNs;
        } else {
            smoothedLoopTimeNs = (ALPHA * loopTimeNs) + ((1.0 - ALPHA) * smoothedLoopTimeNs);
        }

        // Log telemetry automatically if configured
        if (telemetry != null) {
            telemetry.addData("Loop Time (ms)", getLoopTimeMs());
            telemetry.addData("Loop Rate (Hz)", getSmoothedFrequencyHz());
            telemetry.addData("Worst Spike (ms)", getMaxLoopTimeMs());
        }
    }

    /**
     * Resets the tracked peak loop time spike.
     * <p>
     * Useful when transitioning between autonomous phases or TeleOp modes to ignore initial startup spikes.
     */
    public void resetMaxSpike() {
        maxLoopTimeNs = loopTimeNs;
    }

    // ==========================================
    //              GETTERS & METRICS
    // ==========================================

    /**
     * Returns the execution duration of the most recent loop in milliseconds.
     *
     * @return Loop duration in milliseconds (ms).
     */
    public double getLoopTimeMs() {
        return loopTimeNs / 1e6;
    }

    /**
     * Returns the execution duration of the most recent loop in nanoseconds.
     *
     * @return Loop duration in nanoseconds (ns).
     */
    public double getLoopTimeNs() {
        return loopTimeNs;
    }

    /**
     * Returns the highest loop execution spike recorded since {@link #start()} or {@link #resetMaxSpike()}.
     *
     * @return Peak loop spike in milliseconds (ms).
     */
    public double getMaxLoopTimeMs() {
        return maxLoopTimeNs / 1e6;
    }

    /**
     * Returns the smoothed control loop frequency using an Exponential Moving Average (EMA) filter.
     *
     * @return Filtered control loop frequency in Hertz (Hz).
     */
    public double getSmoothedFrequencyHz() {
        return smoothedLoopTimeNs > 0 ? 1e9 / smoothedLoopTimeNs : 0;
    }

    /**
     * Returns the raw, un-filtered instantaneous control loop frequency for the single most recent loop.
     *
     * @return Instantaneous control loop frequency in Hertz (Hz).
     */
    public double getInstantaneousFrequencyHz() {
        return loopTimeNs > 0 ? 1e9 / loopTimeNs : 0;
    }
}