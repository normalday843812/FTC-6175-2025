package org.firstinspires.ftc.teamcode.vision;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@Configurable
public class AprilTagLocalizer {
    private LLResult result;

    private final Limelight3A limelight;

    public AprilTagLocalizer(Limelight3A limelight) {
        this.limelight = limelight;
    }

    public void update(double yawDegField) {
        limelight.updateRobotOrientation(yawDegField);
        result = limelight.getLatestResult();
    }

    public LLResult getResult() {
        return result;
    }

    public long getMillisSinceLastUpdate() {
        return limelight.getTimeSinceLastUpdate();
    }
}