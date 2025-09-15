package org.firstinspires.ftc.teamcode.vision;


import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Configurable
public class AprilTagLocalizer {
    private final OpMode opmode;
    private AprilTagProcessor aprilTag;

    public static Position cameraPosition = new Position(DistanceUnit.METER,
            0, 0, 0, 0); // TODO: Tune
    public static YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0); // Degrees for simplicity TODO: tune

    public AprilTagLocalizer(OpMode opmode) {
        this.opmode = opmode;
    }

    public void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();
    }

    @SuppressLint("DefaultLocale")
    public void addTelemetry(AprilTagDetection detection) {
        if (detection.metadata != null) {
            opmode.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
            opmode.telemetry.addData("Decision margin", detection.decisionMargin);
            opmode.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                    detection.robotPose.getPosition().x,
                    detection.robotPose.getPosition().y,
                    detection.robotPose.getPosition().z));
            opmode.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                    detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                    detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                    detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
        } else {
            opmode.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
            opmode.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
        }
    }

    public AprilTagProcessor getAprilTag() { return aprilTag; }
}