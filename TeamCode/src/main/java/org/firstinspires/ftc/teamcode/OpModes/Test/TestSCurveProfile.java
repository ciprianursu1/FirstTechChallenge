package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.SCurveProfile;
import org.firstinspires.ftc.teamcode.PanelsParameters.ModuleTestParameters;

@TeleOp(name = "Test: S-Curve Profile", group = "Module Tests")
public class TestSCurveProfile extends OpMode {
    private final SCurveProfile profile = new SCurveProfile();
    private final ElapsedTime timer = new ElapsedTime();
    private double lastMaxVel;
    private double lastMaxAccel;
    private double lastMaxJerk;
    private double lastDistance;
    private boolean previousA;

    @Override
    public void init() {
        regenerateProfile();
    }

    @Override
    public void loop() {
        boolean resetRequested = (gamepad1.a || gamepad1.cross) && !previousA;
        if (parametersChanged() || resetRequested) {
            regenerateProfile();
        }
        previousA = gamepad1.a || gamepad1.cross;

        double time = timer.seconds();
        SCurveProfile.ProfileState state = profile.calculate(time);

        telemetry.addLine("S-Curve Profile Test");
        telemetry.addData("Time", "%.3f / %.3f s", time, profile.getTotalTime());
        telemetry.addData("Distance", "%.2f", profile.getDistance());
        telemetry.addData("Position", "%.2f", state.position);
        telemetry.addData("Velocity", "%.2f", state.velocity);
        telemetry.addData("Acceleration", "%.2f", state.acceleration);
        telemetry.addData(
                "Stopping Distance",
                "%.2f",
                SCurveProfile.calculateStoppingDistance(
                        Math.abs(state.velocity),
                        ModuleTestParameters.maxAccel,
                        ModuleTestParameters.maxJerk));
        telemetry.addLine("Edit ModuleTestParameters or press A/CROSS to restart.");
    }

    private boolean parametersChanged() {
        return lastMaxVel != ModuleTestParameters.maxVel
                || lastMaxAccel != ModuleTestParameters.maxAccel
                || lastMaxJerk != ModuleTestParameters.maxJerk
                || lastDistance != ModuleTestParameters.profileDistance;
    }

    private void regenerateProfile() {
        lastMaxVel = ModuleTestParameters.maxVel;
        lastMaxAccel = ModuleTestParameters.maxAccel;
        lastMaxJerk = ModuleTestParameters.maxJerk;
        lastDistance = ModuleTestParameters.profileDistance;
        profile.generate(lastMaxVel, lastMaxAccel, lastMaxJerk, lastDistance);
        timer.reset();
    }
}
