package org.firstinspires.ftc.teamcode.config;

import static org.firstinspires.ftc.teamcode.config.AutoConfig.START_BLUE_AUDIENCE;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.START_BLUE_DEPOT;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.START_RED_AUDIENCE;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.START_RED_DEPOT;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

public final class SimpleAutoConfig {
    private SimpleAutoConfig() {
    }

    public static Pose pickStartPose(boolean isRed, boolean isAudienceSide) {
        if (isRed) return isAudienceSide ? START_RED_AUDIENCE : START_RED_DEPOT;
        return isAudienceSide ? START_BLUE_AUDIENCE : START_BLUE_DEPOT;
    }

    public static Pose pickControlPoint(boolean isRed, boolean isAudienceSide) {
        if (isRed && isAudienceSide) {
            return new Pose(80, 86, 0);
        } else if (!isRed && isAudienceSide) {
            return new Pose(66, 85, 0);
        } else if (isRed) {
            return new Pose(73, 138, 0);
        } else {
            return new Pose(71, 132, 0);
        }
    }

    public static Pose pickEndPose(boolean isRed, boolean isAudienceSide) {
        if (isRed && isAudienceSide) {
            return new Pose(87, 35, 0);
        } else if (!isRed && isAudienceSide) {
            return new Pose(54, 40, Math.toRadians(180));
        } else if (isRed) {
            return new Pose(87, 35, 0);
        } else {
            return new Pose(54, 40, Math.toRadians(180));
        }
    }

    public static Pose pickShootPose(boolean isRed, boolean isAudienceSide) {
        return org.firstinspires.ftc.teamcode.config.AutoDepositConfig.pickShootPose(isRed, isAudienceSide);
    }

    /**
     * Packs poses and prebuilt paths for SimpleAuto.
     */
    public static final class PathPlan {
        public final Pose startPose, controlPoint, shootPose, endPose;
        public final Path toShootPath, returnPath;

        private PathPlan(Pose startPose, Pose controlPoint, Pose shootPose, Pose endPose,
                         Path toShootPath, Path returnPath) {
            this.startPose = startPose;
            this.controlPoint = controlPoint;
            this.shootPose = shootPose;
            this.endPose = endPose;
            this.toShootPath = toShootPath;
            this.returnPath = returnPath;
        }
    }

    //Get some paper and file away the metal that's blocking the ball

    public static PathPlan buildPlan(boolean isRed, boolean isAudienceSide) {
        Pose startPose = pickStartPose(isRed, isAudienceSide);
        Pose controlPoint = pickControlPoint(isRed, isAudienceSide);
        Pose shootPose = pickShootPose(isRed, isAudienceSide);
        Pose endPose = pickEndPose(isRed, isAudienceSide);

        Path toShootPath = new Path(new BezierCurve(startPose, controlPoint, shootPose));
        toShootPath.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

        Path returnPath = new Path(new BezierCurve(shootPose, controlPoint, endPose));
        returnPath.setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading());

        return new PathPlan(startPose, controlPoint, shootPose, endPose, toShootPath, returnPath);
    }
}
