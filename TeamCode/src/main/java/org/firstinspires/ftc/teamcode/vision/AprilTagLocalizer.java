package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.config.LocalisationConfig.TELEMETRY_ENABLED;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

@Configurable
public class AprilTagLocalizer {
    private LLResult result;

    private final Limelight3A limelight;

    private final TelemetryHelper tele;

    public AprilTagLocalizer(Limelight3A limelight, OpMode opmode) {
        this.limelight = limelight;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
    }

    public void update() {
//        limelight.updateRobotOrientation(yawDegField);
        result = limelight.getLatestResult();
        addTelemetry();
    }

    public Pose3D getBotPose() {
        return result.getBotpose();
    }

    public void addTelemetry() {
        Position botPose = result.getBotpose().getPosition();
        tele.addLine("=== LIMELIGHT ===")
                .addData("X", "%.2f", botPose.x)
                .addData("Y", "%.2f", botPose.y)
                .addData("Yaw", "%.2f", result.getBotpose().getOrientation().getYaw());
    }
}