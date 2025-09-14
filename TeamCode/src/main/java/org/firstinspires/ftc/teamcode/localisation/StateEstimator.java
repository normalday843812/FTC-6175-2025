package org.firstinspires.ftc.teamcode.localisation;

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

// TODO: Add localisation via apriltag detection implementation from vision package
public class StateEstimator {
    // Pinpoint
    private final GoBildaPinpointDriver pinpoint;

    // Constructor
    public StateEstimator(GoBildaPinpointDriver pinpoint) {
        this.pinpoint = pinpoint;
    }

    // Must call once per loop for updated data
    public void update() {
        pinpoint.update();
    }

    // Returns field pose in meters and radians
    public Pose2D getPose() {
        double x = pinpoint.getPosX(DistanceUnit.METER);
        double y = pinpoint.getPosY(DistanceUnit.METER);
        double h = pinpoint.getHeading(AngleUnit.RADIANS);
        return new Pose2D(DistanceUnit.METER, x, y, AngleUnit.RADIANS, h);
    }

    public double getHeading() {
        return pinpoint.getHeading(AngleUnit.RADIANS);
    }

    public ChassisSpeeds getChassisSpeedsField() {
        double vx = pinpoint.getVelX(DistanceUnit.METER);
        double vy = pinpoint.getVelY(DistanceUnit.METER);
        double omega = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
        return new ChassisSpeeds(vx, vy, omega);
    }

    public ChassisSpeeds getChassisSpeedsRobot() {
        double vxF = pinpoint.getVelX(DistanceUnit.METER);
        double vyF = pinpoint.getVelY(DistanceUnit.METER);
        double h = pinpoint.getHeading(AngleUnit.RADIANS);
        double cos = Math.cos(h), sin = Math.sin(h);

        double vxR = vxF * cos - vyF * sin;
        double vyR = vxF * sin + vyF * cos;
        double omega = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

        return new ChassisSpeeds(vxR, vyR, omega);
    }

    public void resetPinpoint() {
        pinpoint.resetPosAndIMU();
    }
}
