package org.firstinspires.ftc.teamcode.auto.motion;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

public class MotionController {
    private final Mecanum drive;
    private final Follower follower;
    private final TelemetryHelper tele;
    private final HeadingController manualHeading = new HeadingController();

    public MotionController(Mecanum drive, TelemetryHelper tele) {
        this.drive = drive;
        this.follower = drive.getFollower();
        this.tele = tele;
    }

    public void followToPose(Pose target, double headDeg) {
        Pose current = follower.getPose();

        Pose control = midpointControl(current, target);

        PathChain chain = follower.pathBuilder()
                .addPath(new BezierCurve(follower::getPose, control, target))
                .setLinearHeadingInterpolation(
                        current.getHeading(),
                        Math.toRadians(headDeg),
                        0.8
                )
                .build();

        drive.followPath(chain);
    }

    public void translateFacing(double vxField, double vyField, HeadingTarget target) {
        Pose p = follower.getPose();
        double desiredDeg = target.getTargetHeadingDeg(p);
        double currentDeg = Math.toDegrees(p.getHeading());
        double rot = manualHeading.update(desiredDeg, currentDeg);

        double forward = -vyField;
        double strafe = -vxField;

        drive.setAutoDrive(forward, strafe, rot, true, 0);

        tele.addLine("--- Motion (INTAKE CREEP) ---")
                .addData("HeadTgt", "%.1f", desiredDeg)
                .addData("HeadCur", "%.1f", currentDeg)
                .addData("RotCmd", "%.3f", rot)
                .addData("Vx,Vy", "(%.2f, %.2f)", vxField, vyField);
    }

    public void followToPoseFacingPoint(Pose target, double faceX, double faceY) {
        Pose current = follower.getPose();

        Pose control = midpointControl(current, target);

        PathChain chain = follower.pathBuilder()
                .addPath(new BezierCurve(follower::getPose, control, target))
                .setHeadingInterpolation(
                        HeadingInterpolator.facingPoint(faceX, faceY)
                )
                .build();

        drive.followPath(chain);
    }

    /**
     * @noinspection BooleanMethodIsAlwaysInverted
     */
    public boolean isBusy() {
        return drive.isPathBusy();
    }

    public Pose getPose() {
        return follower.getPose();
    }

    private static Pose midpointControl(Pose start, Pose target) {
        double midX = (start.getX() + target.getX()) / 2.0;
        double midY = (start.getY() + target.getY()) / 2.0;
        return new Pose(midX, midY, target.getHeading());
    }
}
