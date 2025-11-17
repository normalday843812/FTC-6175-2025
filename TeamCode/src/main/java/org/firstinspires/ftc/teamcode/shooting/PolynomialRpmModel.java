package org.firstinspires.ftc.teamcode.shooting;

import static com.pedropathing.math.MathFunctions.getSmallestAngleDifference;
import static org.firstinspires.ftc.teamcode.config.RpmModelConfig.A0;
import static org.firstinspires.ftc.teamcode.config.RpmModelConfig.A1;
import static org.firstinspires.ftc.teamcode.config.RpmModelConfig.A2;
import static org.firstinspires.ftc.teamcode.config.RpmModelConfig.B1;
import static org.firstinspires.ftc.teamcode.config.RpmModelConfig.B2;
import static org.firstinspires.ftc.teamcode.config.RpmModelConfig.MODEL_ENABLED;

import com.pedropathing.geometry.Pose;

/**
 * RPM = a0 + a1*D + a2*D^2 + b1*theta + b2*theta^2
 * D in inches; theta is absolute radians difference to goal.
 */
public final class PolynomialRpmModel implements RpmModel {
    private volatile double a0, a1, a2, b1, b2;
    private volatile boolean valid = false;

    public PolynomialRpmModel() {
        if (MODEL_ENABLED) {
            setCoefficients(
                    A0,
                    A1,
                    A2,
                    B1,
                    B2
            );
        } else {
            valid = false;
        }
    }

    public synchronized void setCoefficients(double a0, double a1, double a2, double b1, double b2) {
        this.a0 = a0;
        this.a1 = a1;
        this.a2 = a2;
        this.b1 = b1;
        this.b2 = b2;
        this.valid = true;
    }

    @Override
    public boolean isValid() {
        return valid;
    }

    @Override
    public double computeTargetRpm(Pose robot, Pose goal) {
        if (!valid) return 0.0;
        double dx = goal.getX() - robot.getX();
        double dy = goal.getY() - robot.getY();
        double dist = Math.hypot(dx, dy);
        double desiredHeading = Math.atan2(dy, dx);
        double theta = Math.abs(getSmallestAngleDifference(robot.getHeading(), desiredHeading));
        return a0 + a1 * dist + a2 * dist * dist + b1 * theta + b2 * theta * theta;
    }
}