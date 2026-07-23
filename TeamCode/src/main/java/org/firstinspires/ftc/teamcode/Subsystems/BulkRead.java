package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/**
 * BulkRead manages manual bulk caching for all connected REV Lynx Hubs
 * (Control Hub and Expansion Hub).
 * <p>
 * Switching to {@link LynxModule.BulkCachingMode#MANUAL} drastically reduces loop times by batching
 * all encoder, digital, and analog reads into a single hardware packet per loop iteration rather than
 * querying hardware over USB/I2C for every individual sensor call.
 */
public class BulkRead {

    /** List of all REV Lynx Modules detected on the robot. */
    private List<LynxModule> hubs;

    /**
     * Initializes manual bulk read caching for all REV Lynx Modules registered in the {@link HardwareMap}.
     * <p>
     * This is the recommended initialization method as it automatically fetches both the
     * Control Hub and Expansion Hub without manual list construction.
     *
     * @param hardwareMap The OpMode {@link HardwareMap} used to retrieve all active {@link LynxModule} instances.
     */
    public void init(HardwareMap hardwareMap) {
        if (hardwareMap != null) {
            init(hardwareMap.getAll(LynxModule.class));
        }
    }

    /**
     * Initializes manual bulk read caching for an explicit list of {@link LynxModule} instances.
     * <p>
     * Sets each hub to {@link LynxModule.BulkCachingMode#MANUAL} and performs an initial cache clear.
     *
     * @param hubs A {@link List} of {@link LynxModule} objects to be managed.
     */
    public void init(List<LynxModule> hubs) {
        this.hubs = hubs;
        if (this.hubs == null) return;

        for (LynxModule lynxModule : this.hubs) {
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            lynxModule.clearBulkCache();
        }
    }

    /**
     * Clears the hardware cache for all configured REV Hubs.
     * <p>
     * <b>CRITICAL:</b> Call this method at the <i>VERY START</i> of every control loop iteration inside
     * {@code while(opModeIsActive())}. The first sensor read after clearing will trigger a single hardware update,
     * while subsequent reads in the same loop will pull instantly from memory.
     */
    public void update() {
        if (hubs == null) return;

        for (LynxModule lynxModule : hubs) {
            lynxModule.clearBulkCache();
        }
    }
}