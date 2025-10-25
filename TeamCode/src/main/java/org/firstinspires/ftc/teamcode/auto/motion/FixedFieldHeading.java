package org.firstinspires.ftc.teamcode.auto.motion;

import com.pedropathing.geometry.Pose;

public class FixedFieldHeading implements HeadingTarget {
    private final double headingDeg;
    private final String name;

    public FixedFieldHeading(double headingDeg) {
        this(headingDeg, "FixedFieldHeading");
    }

    public FixedFieldHeading(double headingDeg, String name) {
        this.headingDeg = headingDeg;
        this.name = name;
    }

    @Override
    public double getTargetHeadingDeg(Pose currentPose) {
        return headingDeg;
    }

    @Override
    public String debugName() {
        return name;
    }
}
