package org.firstinspires.ftc.teamcode.auto.motion;

import static com.pedropathing.math.MathFunctions.clamp;
import static org.firstinspires.ftc.teamcode.config.AutoMotionConfig.DRIVE_APPROACH_GAIN;
import static org.firstinspires.ftc.teamcode.config.AutoMotionConfig.DRIVE_MAX_VEL;
import static org.firstinspires.ftc.teamcode.config.AutoMotionConfig.HEADING_MAX_ROT;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

/**
 * Field-centric: vxField = +X (right), vyField = +Y (forward field).
 */
public class MotionController {
    private final Mecanum drive;
    private final HeadingController headingCtrl;
    private final TelemetryHelper tele;

    public MotionController(Mecanum drive, HeadingController headingCtrl, TelemetryHelper tele) {
        this.drive = drive;
        this.headingCtrl = headingCtrl;
        this.tele = tele;
    }

    public void holdHeading(HeadingTarget target) {
        Pose p = drive.getFollower().getPose();
        double desiredDeg = target.getTargetHeadingDeg(p);
        double currentDeg = Math.toDegrees(p.getHeading());
        double rot = headingCtrl.update(desiredDeg, currentDeg);

        // No translation b/c field-centric is irrelevant
        drive.setAutoDrive(0, 0, rot, true, 0);

        addTelemetry("HOLD", 0, 0, rot,
                desiredDeg, currentDeg, target.debugName());
    }

    public void translateFacing(double vxField, double vyField, HeadingTarget target) {
        Pose p = drive.getFollower().getPose();
        double desiredDeg = target.getTargetHeadingDeg(p);
        double currentDeg = Math.toDegrees(p.getHeading());
        double rot = headingCtrl.update(desiredDeg, currentDeg);

        double forward = clamp(-vyField, -DRIVE_MAX_VEL, DRIVE_MAX_VEL);
        double strafe = clamp(-vxField, -DRIVE_MAX_VEL, DRIVE_MAX_VEL);

        drive.setAutoDrive(forward, strafe, rot, true, 0);

        addTelemetry("TRANSLATE", vxField, vyField, rot,
                desiredDeg, currentDeg, target.debugName());
    }

    public void spinInPlace(double turnCmd) {
        double turn = clamp(turnCmd, -HEADING_MAX_ROT, HEADING_MAX_ROT);
        drive.setAutoDrive(0, 0, turn, true, 0);

        tele.addLine("--- Motion (SPIN) ---")
                .addData("TurnCmd", "%.3f", turn);
    }

    /**
     * Drive to pose with a P-based XY and heading lock.
     * Returns true if finished by distance or timeout.
     */
    public boolean driveToPose(Pose target, double stopDistIn,
                               double timeoutS, HeadingTarget heading) {
        Pose p = drive.getFollower().getPose();
        double dx = target.getX() - p.getX();
        double dy = target.getY() - p.getY();
        double dist = Math.hypot(dx, dy);

        boolean completed = dist < stopDistIn || timeoutS <= 0;

        double desiredDeg = heading.getTargetHeadingDeg(p);
        double currentDeg = Math.toDegrees(p.getHeading());
        double rot = headingCtrl.update(desiredDeg, currentDeg);

        double vx = clamp(dx * DRIVE_APPROACH_GAIN, -DRIVE_MAX_VEL, DRIVE_MAX_VEL);
        double vy = clamp(dy * DRIVE_APPROACH_GAIN, -DRIVE_MAX_VEL, DRIVE_MAX_VEL);

        // Map to drive frame
        double forward = clamp(-vy, -DRIVE_MAX_VEL, DRIVE_MAX_VEL);
        double strafe = clamp(-vx, -DRIVE_MAX_VEL, DRIVE_MAX_VEL);

        if (completed) {
            drive.setAutoDrive(0, 0, rot, true, 0);
        } else {
            drive.setAutoDrive(forward, strafe, rot, true, 0);
        }

        addTelemetry("DRIVE_TO", vx, vy, rot, desiredDeg, currentDeg, heading.debugName());
        return completed;
    }

    private void addTelemetry(String mode, double vxField, double vyField, double rotCmd,
                              double desiredDeg, double currentDeg, String headingName) {
        tele.addLine("--- Motion ---")
                .addData("Mode", mode)
                .addData("HeadTgt", "%.1f", desiredDeg)
                .addData("HeadCur", "%.1f", currentDeg)
                .addData("RotCmd", "%.3f", rotCmd)
                .addData("Vx,Vy", "(%.2f, %.2f)", vxField, vyField)
                .addData("HeadingSrc", headingName);
    }
}
