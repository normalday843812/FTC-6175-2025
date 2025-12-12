package org.firstinspires.ftc.teamcode.config;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.managers.SpindexerModel;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

@Configurable
public class DecodeGameConfig {

    private static final Pose GOAL_BLUE_PEDRO = new Pose(0, 144, 45);
    private static final Pose GOAL_RED_PEDRO = new Pose(144, 144, 0);

    public static double GOAL_BLUE_X = GOAL_BLUE_PEDRO.getX();
    public static double GOAL_BLUE_Y = GOAL_BLUE_PEDRO.getY();
    public static double GOAL_RED_X = GOAL_RED_PEDRO.getX();
    public static double GOAL_RED_Y = GOAL_RED_PEDRO.getY();

    public static Pose[] INTAKE_SETS_RED = new Pose[]{
            new Pose(100, 84, Math.toRadians(0)),
            new Pose(100, 60, Math.toRadians(0)),
            new Pose(100, 35, Math.toRadians(0))
    };
    public static Pose[] INTAKE_SETS_BLUE = new Pose[]{
            new Pose(43, 84, Math.toRadians(180)),
            new Pose(43, 60, Math.toRadians(180)),
            new Pose(43, 35, Math.toRadians(180))
    };

    public static Pose startPose(boolean isRed, boolean audienceSide) {
        return isRed
                ? (audienceSide ? new Pose(88, 7.5, Math.toRadians(90)) : new Pose(117.5, 131.2, Math.toRadians(-142)))
                : (audienceSide ? new Pose(56, 7.5, Math.toRadians(90)) : new Pose(27.29, 131.2, Math.toRadians(-36)));
    }

    public static Pose shootPose(boolean isRed) {
        return isRed ? new Pose(96, 96, Math.toRadians(45))
                : new Pose(44, 100, Math.toRadians(135));
    }

    public static Pose finalPose(boolean isRed) {
        return isRed ? new Pose(106, 12, Math.toRadians(180))
                : new Pose(36, 12, Math.toRadians(0));
    }
}
