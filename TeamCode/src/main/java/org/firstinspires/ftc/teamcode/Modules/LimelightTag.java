package org.firstinspires.ftc.teamcode.Modules;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Collections;
import java.util.List;

/**
 * Convenience wrapper for Limelight AprilTag/fiducial results.
 * <p>
 * This class owns the common Limelight lifecycle calls, selects one fiducial target,
 * exposes the raw Limelight result data, and converts Limelight robot field poses into
 * PedroPathing {@link Pose} values.
 * </p>
 */
public class LimelightTag {
    private static final int NO_TARGET_TAG_ID = -1;
    private static final double INCHES_PER_METER = 39.37007874015748;

    private Limelight3A limelight;
    private LLResult latestResult;
    private LLResultTypes.FiducialResult latestFiducial;
    private PositionRingBuffer.PositionData latestPositionData;

    private int targetTagId = NO_TARGET_TAG_ID;
    private int lastPipeline = -1;
    private boolean useMegaTag2 = false;
    private boolean validResult = false;

    /**
     * Constructs a wrapper around an already-mapped Limelight device.
     *
     * @param limelight Limelight 3A hardware instance.
     */
    public LimelightTag(Limelight3A limelight) {
        this.limelight = limelight;
    }

    /**
     * Constructs a wrapper and retrieves the Limelight from the hardware map.
     *
     * @param hardwareMap FTC hardware map.
     * @param deviceName  Configured Limelight device name.
     */
    public LimelightTag(HardwareMap hardwareMap, String deviceName) {
        this(hardwareMap.get(Limelight3A.class, deviceName));
    }

    /**
     * Starts polling Limelight results.
     */
    public void start() {
        limelight.start();
    }

    /**
     * Switches to a pipeline and starts polling Limelight results.
     *
     * @param pipeline Limelight pipeline index.
     * @return True if the Limelight accepted the pipeline switch.
     */
    public boolean start(int pipeline) {
        boolean switched = switchPipeline(pipeline);
        start();
        return switched;
    }

    /**
     * Pauses Limelight polling without clearing cached results.
     */
    public void pause() {
        limelight.pause();
    }

    /**
     * Stops Limelight polling without clearing cached results.
     */
    public void stop() {
        limelight.stop();
    }

    /**
     * Clears the cached Limelight result, selected tag, and converted pose.
     */
    public void clear() {
        latestResult = null;
        latestFiducial = null;
        latestPositionData = null;
        validResult = false;
    }

    /**
     * Polls the latest Limelight result and updates the selected tag/pose cache.
     *
     * @return True when a valid result and selected tag are available.
     */
    public boolean update() {
        latestResult = limelight.getLatestResult();
        validResult = latestResult != null && latestResult.isValid();
        latestFiducial = null;
        latestPositionData = null;

        if (!validResult) {
            return false;
        }

        latestFiducial = selectFiducial(latestResult.getFiducialResults());
        if (latestFiducial == null) {
            return false;
        }

        Pose pedroPose = getPedroPose();
        if (pedroPose != null) {
            latestPositionData = new PositionRingBuffer.PositionData(pedroPose, getFrameTimestampMs());
        }

        return true;
    }

    /**
     * Switches the Limelight pipeline.
     *
     * @param pipeline Limelight pipeline index.
     * @return True if the Limelight accepted the switch request.
     */
    public boolean switchPipeline(int pipeline) {
        boolean switched = limelight.pipelineSwitch(pipeline);
        if (switched) {
            lastPipeline = pipeline;
        }
        return switched;
    }

    /**
     * Sets the Limelight poll rate.
     *
     * @param pollRateHz Desired poll rate in hertz.
     */
    public void setPollRateHz(int pollRateHz) {
        limelight.setPollRateHz(pollRateHz);
    }

    /**
     * Sends the current robot heading to Limelight for MegaTag2.
     *
     * @param headingDegrees Robot heading in degrees.
     * @return True if the Limelight accepted the orientation update.
     */
    public boolean updateRobotOrientation(double headingDegrees) {
        return limelight.updateRobotOrientation(headingDegrees);
    }

    /**
     * Sets a specific AprilTag ID to track. If unset, the wrapper selects the largest detected tag.
     *
     * @param targetTagId Desired fiducial ID.
     */
    public void setTargetTagId(int targetTagId) {
        this.targetTagId = targetTagId;
    }

    /**
     * Clears the desired tag ID and returns to largest-tag selection.
     */
    public void clearTargetTagId() {
        targetTagId = NO_TARGET_TAG_ID;
    }

    /**
     * Enables or disables MegaTag2 botpose output.
     *
     * @param useMegaTag2 True to read {@link LLResult#getBotpose_MT2()}, false to read {@link LLResult#getBotpose()}.
     */
    public void setUseMegaTag2(boolean useMegaTag2) {
        this.useMegaTag2 = useMegaTag2;
    }

    /**
     * Replaces the wrapped Limelight device and clears cached results.
     *
     * @param limelight Limelight 3A hardware instance.
     */
    public void setLimelight(Limelight3A limelight) {
        this.limelight = limelight;
        clear();
    }

    /**
     * @return Wrapped Limelight instance.
     */
    public Limelight3A getLimelight() {
        return limelight;
    }

    /**
     * @return Latest raw Limelight result, or {@code null} before the first valid poll.
     */
    public LLResult getLatestResult() {
        return latestResult;
    }

    /**
     * @return Selected fiducial result, or {@code null} when no matching tag is visible.
     */
    public LLResultTypes.FiducialResult getLatestFiducial() {
        return latestFiducial;
    }

    /**
     * @return Pedro pose with a latency-adjusted millisecond timestamp, or {@code null} when unavailable.
     */
    public PositionRingBuffer.PositionData getLatestPositionData() {
        return latestPositionData;
    }

    /**
     * @return True when the latest Limelight result is non-null and valid.
     */
    public boolean hasValidResult() {
        return validResult;
    }

    /**
     * @return True when the wrapper has selected a currently visible tag.
     */
    public boolean hasTag() {
        return latestFiducial != null;
    }

    /**
     * @return True when the selected result can provide a robot field pose.
     */
    public boolean hasRobotPose() {
        return latestPositionData != null;
    }

    /**
     * @return True if Limelight polling is running.
     */
    public boolean isRunning() {
        return limelight.isRunning();
    }

    /**
     * @return True when the Limelight connection is active.
     */
    public boolean isConnected() {
        return limelight.isConnected();
    }

    /**
     * @return Desired target tag ID, or -1 when largest-tag selection is enabled.
     */
    public int getTargetTagId() {
        return targetTagId;
    }

    /**
     * @return Last successfully requested pipeline, or -1 if this wrapper has not switched pipelines.
     */
    public int getLastPipeline() {
        return lastPipeline;
    }

    /**
     * @return True when MegaTag2 pose output is enabled.
     */
    public boolean getUseMegaTag2() {
        return useMegaTag2;
    }

    /**
     * @return Number of milliseconds since the Limelight produced a new result.
     */
    public long getTimeSinceLastUpdate() {
        return limelight.getTimeSinceLastUpdate();
    }

    /**
     * @return Latest result timestamp in Control Hub milliseconds, or 0 if no result is cached.
     */
    public long getResultTimestampMs() {
        if (latestResult == null) return 0;
        return latestResult.getControlHubTimeStamp();
    }

    /**
     * @return Estimated image capture timestamp in Control Hub milliseconds, or 0 if no result is cached.
     */
    public long getFrameTimestampMs() {
        if (latestResult == null) return 0;
        return latestResult.getControlHubTimeStamp() - Math.round(getTotalLatencyMs());
    }

    /**
     * @return Latest result staleness in milliseconds, or 0 if no result is cached.
     */
    public long getStalenessMs() {
        if (latestResult == null) return 0;
        return latestResult.getStaleness();
    }

    /**
     * @return Limelight capture latency in milliseconds.
     */
    public double getCaptureLatencyMs() {
        if (latestResult == null) return 0;
        return latestResult.getCaptureLatency();
    }

    /**
     * @return Limelight targeting latency in milliseconds.
     */
    public double getTargetingLatencyMs() {
        if (latestResult == null) return 0;
        return latestResult.getTargetingLatency();
    }

    /**
     * @return Control Hub JSON parse latency in milliseconds.
     */
    public double getParseLatencyMs() {
        if (latestResult == null) return 0;
        return latestResult.getParseLatency();
    }

    /**
     * @return Capture plus targeting latency in milliseconds.
     */
    public double getTotalLatencyMs() {
        return getCaptureLatencyMs() + getTargetingLatencyMs();
    }

    /**
     * @return Current pipeline index reported by the latest result, or -1 if no result is cached.
     */
    public int getPipelineIndex() {
        if (latestResult == null) return -1;
        return latestResult.getPipelineIndex();
    }

    /**
     * @return Current pipeline type reported by the latest result.
     */
    public String getPipelineType() {
        if (latestResult == null) return "";
        return latestResult.getPipelineType();
    }

    /**
     * @return Number of fiducials in the latest result.
     */
    public int getFiducialCount() {
        return getFiducialResults().size();
    }

    /**
     * @return Number of tags used by Limelight's botpose solution.
     */
    public int getBotposeTagCount() {
        if (latestResult == null) return 0;
        return latestResult.getBotposeTagCount();
    }

    /**
     * @return Span of tags used by Limelight's botpose solution in meters.
     */
    public double getBotposeSpan() {
        if (latestResult == null) return 0;
        return latestResult.getBotposeSpan();
    }

    /**
     * @return Average distance to tags used by Limelight's botpose solution in meters.
     */
    public double getBotposeAvgDist() {
        if (latestResult == null) return 0;
        return latestResult.getBotposeAvgDist();
    }

    /**
     * @return Average tag area used by Limelight's botpose solution.
     */
    public double getBotposeAvgArea() {
        if (latestResult == null) return 0;
        return latestResult.getBotposeAvgArea();
    }

    /**
     * @return Horizontal offset of the primary target from the crosshair in degrees.
     */
    public double getTx() {
        if (latestResult == null) return 0;
        return latestResult.getTx();
    }

    /**
     * @return Vertical offset of the primary target from the crosshair in degrees.
     */
    public double getTy() {
        if (latestResult == null) return 0;
        return latestResult.getTy();
    }

    /**
     * @return Horizontal offset of the primary target from the principal pixel in degrees.
     */
    public double getTxNC() {
        if (latestResult == null) return 0;
        return latestResult.getTxNC();
    }

    /**
     * @return Vertical offset of the primary target from the principal pixel in degrees.
     */
    public double getTyNC() {
        if (latestResult == null) return 0;
        return latestResult.getTyNC();
    }

    /**
     * @return Area of the primary target as a percentage of the image.
     */
    public double getTa() {
        if (latestResult == null) return 0;
        return latestResult.getTa();
    }

    /**
     * @return Selected tag ID, or -1 when no tag is selected.
     */
    public int getTagId() {
        if (latestFiducial == null) return NO_TARGET_TAG_ID;
        return latestFiducial.getFiducialId();
    }

    /**
     * @return Selected tag family string.
     */
    public String getFamily() {
        if (latestFiducial == null) return "";
        return latestFiducial.getFamily();
    }

    /**
     * @return Selected tag area as a percentage of the image.
     */
    public double getTargetArea() {
        if (latestFiducial == null) return 0;
        return latestFiducial.getTargetArea();
    }

    /**
     * @return Selected tag horizontal offset from the crosshair in pixels.
     */
    public double getTargetXPixels() {
        if (latestFiducial == null) return 0;
        return latestFiducial.getTargetXPixels();
    }

    /**
     * @return Selected tag vertical offset from the crosshair in pixels.
     */
    public double getTargetYPixels() {
        if (latestFiducial == null) return 0;
        return latestFiducial.getTargetYPixels();
    }

    /**
     * @return Selected tag horizontal offset from the crosshair in degrees.
     */
    public double getTargetXDegrees() {
        if (latestFiducial == null) return 0;
        return latestFiducial.getTargetXDegrees();
    }

    /**
     * @return Selected tag vertical offset from the crosshair in degrees.
     */
    public double getTargetYDegrees() {
        if (latestFiducial == null) return 0;
        return latestFiducial.getTargetYDegrees();
    }

    /**
     * @return Selected tag horizontal offset from the principal pixel in degrees.
     */
    public double getTargetXDegreesNoCrosshair() {
        if (latestFiducial == null) return 0;
        return latestFiducial.getTargetXDegreesNoCrosshair();
    }

    /**
     * @return Selected tag vertical offset from the principal pixel in degrees.
     */
    public double getTargetYDegreesNoCrosshair() {
        if (latestFiducial == null) return 0;
        return latestFiducial.getTargetYDegreesNoCrosshair();
    }

    /**
     * @return Selected tag skew value.
     */
    public double getSkew() {
        if (latestFiducial == null) return 0;
        return latestFiducial.getSkew();
    }

    /**
     * @return Selected tag corners as [x, y] pixel pairs.
     */
    public List<List<Double>> getTargetCorners() {
        if (latestFiducial == null) return Collections.emptyList();
        return latestFiducial.getTargetCorners();
    }

    /**
     * @return All fiducial results from the latest Limelight result.
     */
    public List<LLResultTypes.FiducialResult> getFiducialResults() {
        if (latestResult == null) return Collections.emptyList();
        return latestResult.getFiducialResults();
    }

    /**
     * @return Limelight botpose as a raw FTC {@link Pose3D}, or {@code null} when unavailable.
     */
    public Pose3D getBotpose3D() {
        if (latestResult == null) return null;
        return useMegaTag2 ? latestResult.getBotpose_MT2() : latestResult.getBotpose();
    }

    /**
     * @return Limelight botpose converted to a PedroPathing pose, or {@code null} when unavailable.
     */
    public Pose getPedroPose() {
        Pose3D botpose = getBotpose3D();
        if (botpose == null) return null;
        return botPoseToPedro(botpose);
    }

    /**
     * @return Selected tag's camera pose in target space, or {@code null} when unavailable.
     */
    public Pose3D getCameraPoseTargetSpace() {
        if (latestFiducial == null) return null;
        return latestFiducial.getCameraPoseTargetSpace();
    }

    /**
     * @return Selected tag's robot pose in field space, or {@code null} when unavailable.
     */
    public Pose3D getRobotPoseFieldSpace() {
        if (latestFiducial == null) return null;
        return latestFiducial.getRobotPoseFieldSpace();
    }

    /**
     * @return Selected tag's robot pose in target space, or {@code null} when unavailable.
     */
    public Pose3D getRobotPoseTargetSpace() {
        if (latestFiducial == null) return null;
        return latestFiducial.getRobotPoseTargetSpace();
    }

    /**
     * @return Selected tag's target pose in camera space, or {@code null} when unavailable.
     */
    public Pose3D getTargetPoseCameraSpace() {
        if (latestFiducial == null) return null;
        return latestFiducial.getTargetPoseCameraSpace();
    }

    /**
     * @return Selected tag's target pose in robot space, or {@code null} when unavailable.
     */
    public Pose3D getTargetPoseRobotSpace() {
        if (latestFiducial == null) return null;
        return latestFiducial.getTargetPoseRobotSpace();
    }

    private LLResultTypes.FiducialResult selectFiducial(List<LLResultTypes.FiducialResult> fiducials) {
        LLResultTypes.FiducialResult best = null;

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (targetTagId != NO_TARGET_TAG_ID && fiducial.getFiducialId() != targetTagId) {
                continue;
            }

            if (best == null || fiducial.getTargetArea() > best.getTargetArea()) {
                best = fiducial;
            }
        }

        return best;
    }

    private Pose botPoseToPedro(Pose3D pose) {
        Pose2D pose2D = new Pose2D(
                DistanceUnit.INCH,
                pose.getPosition().x * INCHES_PER_METER,
                pose.getPosition().y * INCHES_PER_METER,
                AngleUnit.RADIANS,
                pose.getOrientation().getYaw(AngleUnit.RADIANS));

        return PoseConverter.pose2DToPose(pose2D, InvertedFTCCoordinates.INSTANCE)
                .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
}
