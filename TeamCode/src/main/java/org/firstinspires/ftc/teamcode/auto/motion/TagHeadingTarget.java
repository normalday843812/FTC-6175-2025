package org.firstinspires.ftc.teamcode.auto.motion;

import static org.firstinspires.ftc.teamcode.config.TagGeometryConfig.METERS_PER_POSE_UNIT;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

/**
 * Pure geometry helper: bearing from robot pose to a field tag, in degrees.
 */
public final class TagHeadingTarget {
    private TagHeadingTarget() {
    }

    public static double getBearingDeg(int tagId, Pose robotPose) {
        AprilTagMetadata md = AprilTagGameDatabase.getCurrentGameTagLibrary().lookupTag(tagId);
        if (md == null) return 0.0;

        VectorF p = md.fieldPosition; // meters
        double tagXm = p.get(0);
        double tagYm = p.get(1);

        // Convert robot pose to meters so units match
        double rxm = robotPose.getX() * METERS_PER_POSE_UNIT;
        double rym = robotPose.getY() * METERS_PER_POSE_UNIT;

        double dx = tagXm - rxm;
        double dy = tagYm - rym;

        return Math.toDegrees(Math.atan2(dy, dx));
    }
}
