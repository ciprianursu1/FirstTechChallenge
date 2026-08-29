package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.LimelightTag;
import org.firstinspires.ftc.teamcode.PanelsParameters.ModuleTestParameters;

@TeleOp(name = "Test: Limelight Tag", group = "Module Tests")
public class TestLimelightTag extends OpMode {
    private LimelightTag limelightTag;

    @Override
    public void init() {
        limelightTag = new LimelightTag(hardwareMap, ModuleTestParameters.limelightName);
        limelightTag.setUseMegaTag2(ModuleTestParameters.useMegaTag2);
        if (ModuleTestParameters.limelightTargetTagId >= 0) {
            limelightTag.setTargetTagId(ModuleTestParameters.limelightTargetTagId);
        } else {
            limelightTag.clearTargetTagId();
        }
        limelightTag.start(ModuleTestParameters.limelightPipeline);
    }

    @Override
    public void loop() {
        limelightTag.setUseMegaTag2(ModuleTestParameters.useMegaTag2);
        if (ModuleTestParameters.limelightTargetTagId >= 0) {
            limelightTag.setTargetTagId(ModuleTestParameters.limelightTargetTagId);
        } else {
            limelightTag.clearTargetTagId();
        }
        if (ModuleTestParameters.useMegaTag2) {
            limelightTag.updateRobotOrientation(ModuleTestParameters.robotHeadingDegrees);
        }

        boolean selectedTagVisible = limelightTag.update();

        telemetry.addLine("LimelightTag Test");
        telemetry.addData("Connected", limelightTag.isConnected());
        telemetry.addData("Running", limelightTag.isRunning());
        telemetry.addData("Valid Result", limelightTag.hasValidResult());
        telemetry.addData("Selected Tag Visible", selectedTagVisible);
        telemetry.addData("Pipeline", "%d / %s", limelightTag.getPipelineIndex(), limelightTag.getPipelineType());
        telemetry.addData("Fiducials", limelightTag.getFiducialCount());
        telemetry.addData("Tag ID", limelightTag.getTagId());
        telemetry.addData("Family", limelightTag.getFamily());
        telemetry.addData("tx / ty", "%.2f / %.2f deg", limelightTag.getTargetXDegrees(), limelightTag.getTargetYDegrees());
        telemetry.addData("Area", "%.3f", limelightTag.getTargetArea());
        telemetry.addData("Staleness", "%d ms", limelightTag.getStalenessMs());
        telemetry.addData("Latency", "%.2f ms", limelightTag.getTotalLatencyMs());
        telemetry.addData("Frame Timestamp", limelightTag.getFrameTimestampMs());
        telemetry.addData("Pedro Pose", limelightTag.getPedroPose());
        telemetry.addData("Position Data", limelightTag.getLatestPositionData() != null);
        telemetry.addData("Botpose Tags", limelightTag.getBotposeTagCount());
        telemetry.addData("Botpose Avg Dist", "%.2f m", limelightTag.getBotposeAvgDist());
    }

    @Override
    public void stop() {
        if (limelightTag != null) {
            limelightTag.stop();
        }
    }
}
