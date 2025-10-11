package org.firstinspires.ftc.teamcode.vision;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
@Configurable
public class AprilTagLocalizerLimelight {
    private LLResult result;

    private final Limelight3A limelight;
    public AprilTagLocalizerLimelight(Limelight3A limelight) {
        this.limelight = limelight;
    }

    public void update(double yawDeg) {
        limelight.updateRobotOrientation(yawDeg);
        result = limelight.getLatestResult();
    }

    public LLResult getResult() { return result; }
}