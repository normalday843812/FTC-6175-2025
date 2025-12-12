package org.firstinspires.ftc.teamcode.auto.motion;

import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.GOAL_BLUE_X;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.GOAL_BLUE_Y;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.GOAL_RED_X;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.GOAL_RED_Y;

import com.pedropathing.geometry.Pose;

public class AllianceGoalHeadingTarget implements HeadingTarget {
    private final double goalX;
    private final double goalY;

    public AllianceGoalHeadingTarget(boolean isRed) {
        this.goalX = isRed ? GOAL_RED_X : GOAL_BLUE_X;
        this.goalY = isRed ? GOAL_RED_Y : GOAL_BLUE_Y;
    }

    @Override
    public double getTargetHeadingDeg(Pose currentPose) {
        double dx = goalX - currentPose.getX();
        double dy = goalY - currentPose.getY();

        return Math.toDegrees(Math.atan2(dy, dx));
    }

    @Override
    public String debugName() {
        return "AllianceGoalHeadingTarget";
    }
}
