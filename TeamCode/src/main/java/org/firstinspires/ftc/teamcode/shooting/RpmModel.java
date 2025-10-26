package org.firstinspires.ftc.teamcode.shooting;

import com.pedropathing.geometry.Pose;

public interface RpmModel {
    boolean isValid();

    double computeTargetRpm(Pose robot, Pose goal);
}