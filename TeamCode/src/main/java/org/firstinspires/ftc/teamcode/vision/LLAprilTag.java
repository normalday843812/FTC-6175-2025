package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_BLUE;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_GPP;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_PGP;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_PPG;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_RED;
import static org.firstinspires.ftc.teamcode.config.LLAprilTagConfig.TELEMETRY_ENABLED;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

import java.util.List;

public class LLAprilTag {
    public static Integer SELECTED_PATTERN_TAG_ID = null;

    private final Limelight3A limelight;
    private final TelemetryHelper tele;

    private double selectedPatternConfidence = 0.0;

    private LLResult result;

    public LLAprilTag(Limelight3A limelight, OpMode opMode) {
        this.limelight = limelight;
        this.tele = new TelemetryHelper(opMode, TELEMETRY_ENABLED);
    }

    public void update() {
        result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            addTelemetry();
            return;
        }

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags != null) {
            for (LLResultTypes.FiducialResult f : tags) {
                int id = f.getFiducialId();

                // Track only these pattern IDs, keep the highest-confidence seen
                if (id == APRIL_TAG_GPP || id == APRIL_TAG_PGP || id == APRIL_TAG_PPG) {
                    double conf = confidenceFromDistance(f);
                    if (conf > selectedPatternConfidence) {
                        selectedPatternConfidence = conf;
                        SELECTED_PATTERN_TAG_ID = id;
                    }
                }
            }
        }

        addTelemetry();
    }

    public Pose3D getBotPose() {
        return (result != null && result.isValid()) ? result.getBotpose() : null;
    }

    private double confidenceFromDistance(LLResultTypes.FiducialResult f) {
        Pose3D cam = f.getTargetPoseCameraSpace();
        if (cam == null || cam.getPosition() == null) return 0.0;
        double x = cam.getPosition().x;
        double y = cam.getPosition().y;
        double z = cam.getPosition().z;
        double meters = Math.sqrt(x * x + y * y + z * z);
        return 1.0 / (1.0 + meters);
    }

    private void addTelemetry() {
        boolean poseAvailable = result != null && result.getBotpose() != null;

        tele.addLine("=== LL APRILTAG ===")
                .addData("Pattern ID", () -> SELECTED_PATTERN_TAG_ID)
                .addData("Pattern Confidence", "%.3f", selectedPatternConfidence)
                .addData("Pose Set", poseAvailable ? "yes" : "no");

        if (result != null) {
            Pose3D botpose = result.getBotpose();
            tele.addData("LL x", "%.2f", botpose.getPosition().x)
                .addData("LL y", "%.2f", botpose.getPosition().y)
                .addData("LL yaw", "%.2f", botpose.getOrientation().getYaw(AngleUnit.DEGREES));
            boolean sawBlueOrRed = false;
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            if (tags != null) {
                for (LLResultTypes.FiducialResult f : tags) {
                    int id = f.getFiducialId();
                    if (id == APRIL_TAG_BLUE || id == APRIL_TAG_RED) {
                        sawBlueOrRed = true;
                        break;
                    }
                }
            }
            tele.addData("Saw home tag", sawBlueOrRed ? "yes" : "no");
        }
    }
}
