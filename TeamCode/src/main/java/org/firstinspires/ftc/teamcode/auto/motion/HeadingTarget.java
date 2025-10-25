package org.firstinspires.ftc.teamcode.auto.motion;

import com.pedropathing.geometry.Pose;

public interface HeadingTarget {
    double getTargetHeadingDeg(Pose currentPose);
    String debugName();
}
