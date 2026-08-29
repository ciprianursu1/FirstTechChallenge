package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.PositionRingBuffer;
import org.firstinspires.ftc.teamcode.PanelsParameters.ModuleTestParameters;

@TeleOp(name = "Test: Position Ring Buffer", group = "Module Tests")
public class TestPositionRingBuffer extends OpMode {
    private PositionRingBuffer ringBuffer;
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        ringBuffer = new PositionRingBuffer(ModuleTestParameters.ringBufferSize);
        timer.reset();
    }

    @Override
    public void loop() {
        double time = timer.seconds();
        Pose currentPose = new Pose(
                24.0 * time,
                12.0 * Math.sin(time),
                time % (2.0 * Math.PI));

        long nowMs = System.currentTimeMillis();
        long queryTimestampMs =
                nowMs - Math.round(ModuleTestParameters.ringBufferQueryLatencyMs);

        ringBuffer.update(currentPose);
        PositionRingBuffer.PositionData closest = ringBuffer.getClosestPose(queryTimestampMs);
        Pose interpolated = ringBuffer.getInterpolatedPose(queryTimestampMs);

        telemetry.addLine("PositionRingBuffer Test");
        telemetry.addData("Now", nowMs);
        telemetry.addData("Query Timestamp", queryTimestampMs);
        telemetry.addData("Generated Pose", currentPose.toString());
        telemetry.addData("Closest", closest == null ? "null" : closest.pose.toString());
        telemetry.addData("Closest Timestamp", closest == null ? 0 : closest.timestamp);
        telemetry.addData("Interpolated", interpolated == null ? "null" : interpolated.toString());
        telemetry.addLine("Increase query latency to test deeper history.");
    }
}
