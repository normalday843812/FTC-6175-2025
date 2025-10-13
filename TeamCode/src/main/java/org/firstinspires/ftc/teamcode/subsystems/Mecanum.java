package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.DriveConfig.KD_YAW;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.KP_YAW;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.OMEGA_MAX;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.ROT_DB;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.SLOW_MODE_FACTOR;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.STICK_DB;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.util.MathUtil.deadband;
import static org.firstinspires.ftc.teamcode.util.MathUtil.wrapRad;

import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.localisation.LocalisationConstants;
import org.firstinspires.ftc.teamcode.localisation.StateEstimator;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

public class Mecanum {
    // State estimation
    private final Follower follower;
    private final StateEstimator state;

    // Controls
    private final GamepadMap map;

    // Telemetry
    private final TelemetryHelper tele;

    // Angle and slow mode modifiers
    private boolean angleLock = false, slowMode = false, fieldCentricEnabled = false;
    private double targetHeading = 0.0;
    private boolean teleopStarted = false;

    // Drivetrain bridge
    public static class PedroMecanumDrivetrain extends Drivetrain {
        private final DcMotorEx fl, fr, bl, br;
        private double maxPower = 1.0;
        // These must store calibrated max speeds (in/s)
        private double xVel, yVel;

        public PedroMecanumDrivetrain(DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, DcMotorEx br) {
            this.fl = fl;
            this.fr = fr;
            this.bl = bl;
            this.br = br;
        }

        @Override
        public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
            double xF = correctivePower.getXComponent() + pathingPower.getXComponent();
            double yF = correctivePower.getYComponent() + pathingPower.getYComponent();

            double cos = Math.cos(robotHeading), sin = Math.sin(robotHeading);
            double xR = xF * cos + yF * sin;
            double yR = -xF * sin + yF * cos;

            double sign = Math.cos(headingPower.getTheta() - robotHeading) >= 0 ? 1 : -1;
            double turn = headingPower.getMagnitude() * sign;

            double flP = yR + xR + turn;
            double blP = yR - xR + turn;
            double frP = yR - xR - turn;
            double brP = yR + xR - turn;

            double maxMag = Math.max(1.0, Math.max(Math.abs(flP),
                    Math.max(Math.abs(frP), Math.max(Math.abs(blP), Math.abs(brP)))));
            return new double[]{
                    (flP / maxMag) * maxPower, (frP / maxMag) * maxPower,
                    (blP / maxMag) * maxPower, (brP / maxMag) * maxPower
            };
        }

        @Override
        public void runDrive(double[] p) {
            fl.setPower(p[0]);
            fr.setPower(p[1]);
            bl.setPower(p[2]);
            br.setPower(p[3]);
        }

        @Override public void updateConstants() {}
        @Override public void breakFollowing() {}

        @Override public void startTeleopDrive() {}
        @Override public void startTeleopDrive(boolean brakeMode) {}

        @Override public double xVelocity() { return xVel; }
        @Override public double yVelocity() { return yVel; }
        @Override public void setXVelocity(double v) { xVel = v; }
        @Override public void setYVelocity(double v) { yVel = v; }

        @Override public void setMaxPowerScaling(double s) { maxPower = Math.max(0, Math.min(1, s)); }
        @Override public double getMaxPowerScaling() { return maxPower; }

        @Override public double getVoltage() { return 12.0; } // optional voltage compensation hook
        @Override public String debugString() { return "mecanum"; }
    }

    public Mecanum(StateEstimator state,
                   GamepadMap map,
                   OpMode opmode,
                   Follower follower) {
        this.follower = follower;
        this.state = state;
        this.map = map;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
    }

    public void operate() {
        handleToggles();

        if (!teleopStarted) {
            follower.startTeleopDrive();
            teleopStarted = true;
        }

        double vx = deadband(map.forward, STICK_DB);
        double vy = deadband(map.strafe, STICK_DB);
        double rotateStick = map.rotate;

        boolean driverRotating = Math.abs(rotateStick) > ROT_DB;
        double omegaCmd;

        if (driverRotating) {
            omegaCmd = rotateStick;
            targetHeading = follower.getPose().getHeading();
        } else if (angleLock) {
            double heading = follower.getPose().getHeading();
            double error = wrapRad(targetHeading - heading);
            double omega = follower.getAngularVelocity();
            omegaCmd = KP_YAW * error - KD_YAW * omega;
            omegaCmd = Math.max(-OMEGA_MAX, Math.min(OMEGA_MAX, omegaCmd));
        } else {
            omegaCmd = 0.0;
        }

        if (slowMode) {
            vx *= SLOW_MODE_FACTOR;
            vy *= SLOW_MODE_FACTOR;
            omegaCmd *= SLOW_MODE_FACTOR;
            follower.setMaxPowerScaling(SLOW_MODE_FACTOR);
        } else {
            follower.setMaxPowerScaling(1.0);
        }

        boolean isRobotCentric = !fieldCentricEnabled;
        follower.setTeleOpDrive(vx, vy, omegaCmd, isRobotCentric);
        follower.update();

        addTelemetry(vx, vy, omegaCmd, isRobotCentric);
    }

    private void handleToggles() {
        if (map.angleLockToggle) {
            angleLock = !angleLock;
            if (angleLock) targetHeading = follower.getPose().getHeading();
        }
        if (map.slowModeToggle) slowMode = !slowMode;
        if (map.fieldCentricToggle) fieldCentricEnabled = !fieldCentricEnabled;
        if (map.stateEstimatorFallbackToggle) state.setVisionEnabled(!state.isVisionEnabled());
        if (map.resetPinpointButton) resetFieldCentric();
    }

    private void resetFieldCentric() {
        state.resetIMU();
        state.setStartPose(new Pose(0, 0, 0));
        targetHeading = 0.0;
    }

    private void addTelemetry(double vx, double vy, double omegaCmd, boolean isRobotCentric) {
        Pose pIn = follower.getPose();
        Vector vIn = follower.getVelocity();
        double headingDeg = Math.toDegrees(pIn.getHeading());

        tele.addLine("--- Mecanum [Pedro TeleOp] ---")
                .addData("Mode", "%s", isRobotCentric ? "Robot" : "Field")
                .addData("Toggles", "slow=%b angleLock=%b", slowMode, angleLock)
                .addData("Cmd", "vx=%.2f vy=%.2f ω=%.2f", vx, vy, omegaCmd)
                .addData("Pose_in", "(%.1f, %.1f) | %.1f°", pIn.getX(), pIn.getY(), headingDeg)
                .addData("Pose_m", "(%.3f, %.3f) | %.1f°", pIn.getX() * LocalisationConstants.IN_TO_M, pIn.getY() * LocalisationConstants.IN_TO_M, headingDeg)
                .addData("Vel_in/s", "vx=%.2f vy=%.2f", vIn.getXComponent(), vIn.getYComponent())
                .addData("HeadingErr_deg", "%.2f", Math.toDegrees(wrapRad(targetHeading - pIn.getHeading())))
                .addData("Follower", "busy=%b teleop=%b stuck=%b",
                        follower.isBusy(), follower.isTeleopDrive(), follower.isRobotStuck())
                .addData("MaxPower", "%.2f", follower.getMaxPowerScaling());
    }
}