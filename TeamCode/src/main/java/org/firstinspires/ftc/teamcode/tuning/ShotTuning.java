package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.teamcode.config.ShooterConfig.*;
import static org.firstinspires.ftc.teamcode.tuning.ShotTuningConstants.DRIVE_SCALE;
import static org.firstinspires.ftc.teamcode.tuning.ShotTuningConstants.HOOD_LIST;
import static org.firstinspires.ftc.teamcode.tuning.ShotTuningConstants.LIMELIGHT_PIPELINE;
import static org.firstinspires.ftc.teamcode.tuning.ShotTuningConstants.MAX_OMEGA_RAD;
import static org.firstinspires.ftc.teamcode.tuning.ShotTuningConstants.MAX_VEL_MPS;
import static org.firstinspires.ftc.teamcode.tuning.ShotTuningConstants.RPM_LIST;
import static org.firstinspires.ftc.teamcode.tuning.ShotTuningConstants.ROTATE_SCALE;
import static org.firstinspires.ftc.teamcode.tuning.ShotTuningConstants.TELEMETRY_ROWS;
import static org.firstinspires.ftc.teamcode.tuning.ShotTuningConstants.YAW_KD;
import static org.firstinspires.ftc.teamcode.tuning.ShotTuningConstants.YAW_KP;

import android.annotation.SuppressLint;

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.localisation.StateEstimator;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalizerLimelight;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "Shot Tuning", group = "Tuning")
public class ShotTuning extends LinearOpMode {

    // MADE BY CHATGPT MAY NOT WORK

    // Hardware / subsystems
    private RobotHardware hw;
    private StateEstimator state;
    private AprilTagLocalizerLimelight ll;

    private DcMotorEx fl, fr, bl, br;

    // Controls (rising edges)
    private final Edge edgeA = new Edge();   // log shot
    private final Edge edgeX = new Edge();   // undo last
    private final Edge edgeY = new Edge();   // skip to next cell
    private final Edge edgeRB = new Edge();  // toggle yaw lock
    private final Edge edgeDUp = new Edge();
    private final Edge edgeDDown = new Edge();
    private final Edge edgeDLeft = new Edge();
    private final Edge edgeDRight = new Edge();

    private boolean aimLock = true;
    private int i = 0, j = 0;

    // Latest Limelight sample
    private boolean llValid = false;
    private double llTxDeg = Double.NaN;

    // Log row
    private static class Row {
        int i, j;
        double rpm_t, rpm_m, hood, x, y, th_deg, psi_deg, V;
        boolean seen;
    }
    private final List<Row> log = new ArrayList<>();

    @Override
    public void runOpMode() {
        // Init hardware
        hw = new RobotHardware(this);
        hw.initDriveMotors();
        fl = hw.getFrontLeft();
        fr = hw.getFrontRight();
        bl = hw.getBackLeft();
        br = hw.getBackRight();

        hw.initPinpoint();
        hw.initLimeLight(100);
        hw.setLimelightPipeline(LIMELIGHT_PIPELINE);

        ll = new AprilTagLocalizerLimelight(hw.getLimelight());
        state = new StateEstimator(this, hw.getPinpoint(), ll);

        Shooter shooter = new Shooter(hw.getShooterMotor(), new GamepadMap(this), this);
        hw.initHood();

        if (isStopRequested()) return;
        waitForStart();

        // Shooter into velocity mode with your PIDF
        hw.getShooterMotor().setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.getShooterMotor().setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        hw.getShooterMotor().setVelocityPIDFCoefficients(P, I, D, F);

        while (opModeIsActive()) {
            state.update();            // updates Pinpoint and pushes yaw to Limelight
            updateLimelightSample();   // reads LLResult

            // Grid selection on D-Pad (edge-triggered)
            if (edgeDUp.press(gamepad1.dpad_up))       i = clampIndex(i + 1, RPM_LIST.length);
            if (edgeDDown.press(gamepad1.dpad_down))   i = clampIndex(i - 1, RPM_LIST.length);
            if (edgeDRight.press(gamepad1.dpad_right)) j = clampIndex(j + 1, HOOD_LIST.length);
            if (edgeDLeft.press(gamepad1.dpad_left))   j = clampIndex(j - 1, HOOD_LIST.length);

            // Apply current setpoint
            double rpmTarget = RPM_LIST[i];
            shooter.setRpm(rpmTarget);
            hw.getHoodServo().setPosition(Range.clip(HOOD_LIST[j], 0.0, 1.0));

            // Drive with optional yaw lock on tx
            driveLoop();

            // Log shot on A when settled
            if (edgeA.press(gamepad1.a) && isSettled()) {
                log.add(snapshot(rpmTarget));
                stepIndexRowMajor(); // auto-advance
            }

            // Undo last
            if (edgeX.press(gamepad1.x) && !log.isEmpty()) {
                log.remove(log.size() - 1);
            }

            // Skip to next cell
            if (edgeY.press(gamepad1.y)) {
                stepIndexRowMajor();
            }

            // Toggle yaw lock
            if (edgeRB.press(gamepad1.right_bumper)) {
                aimLock = !aimLock;
            }

            if (TELEMETRY_ENABLED) {
                addTelemetry(rpmTarget);
            }
            idle();
        }
    }

    private void updateLimelightSample() {
        LLResult r = ll.getResult();
        llValid = (r != null && r.isValid());
        llTxDeg = llValid ? r.getTx() : Double.NaN;
    }

    private boolean isSettled() {
        ChassisSpeeds vf = state.getChassisSpeedsField();
        return Math.hypot(vf.vxMetersPerSecond, vf.vyMetersPerSecond) < MAX_VEL_MPS
                && Math.abs(vf.omegaRadiansPerSecond) < MAX_OMEGA_RAD;
    }

    private Row snapshot(double rpmTarget) {
        Row row = new Row();
        row.i = i;
        row.j = j;
        row.rpm_t = rpmTarget;

        double tps = hw.getShooterMotor().getVelocity(); // ticks/s at motor encoder
        row.rpm_m = tps * 60.0 / TPR_MOTOR;

        Pose2D pose = state.getPose(); // odometry-frame is fine for logging
        row.x = pose.getX(DistanceUnit.METER);
        row.y = pose.getY(DistanceUnit.METER);
        row.th_deg = Math.toDegrees(pose.getHeading(AngleUnit.RADIANS));

        row.hood = HOOD_LIST[j];
        row.seen = llValid;
        row.psi_deg = llValid ? llTxDeg : Double.NaN;
        row.V = batteryV();
        return row;
    }

    private void driveLoop() {
        // Sticks
        double fwd = deadband(-gamepad1.left_stick_y) * DRIVE_SCALE;
        double str = deadband(-gamepad1.left_stick_x) * DRIVE_SCALE;
        double rotStick = deadband(-gamepad1.right_stick_x) * ROTATE_SCALE;

        // Field->robot transform
        double h = state.getHeading();
        double cos = Math.cos(h), sin = Math.sin(h);
        double vxR = fwd * cos + str * sin;
        double vyR = -fwd * sin + str * cos;

        // Yaw command: manual if rotating or no vision/lock; else PD on tx
        double omegaCmd;
        boolean driverRot = Math.abs(rotStick) > 0.01;
        if (driverRot || !aimLock || !llValid) {
            omegaCmd = rotStick;
        } else {
            double errRad = Math.toRadians(llTxDeg); // +tx = target to right
            ChassisSpeeds vr = state.getChassisSpeedsRobot();
            omegaCmd = clamp(YAW_KP * errRad - YAW_KD * vr.omegaRadiansPerSecond);
        }

        // Mecanum mix
        double flP = vyR + vxR + omegaCmd;
        double blP = vyR - vxR + omegaCmd;
        double frP = vyR - vxR - omegaCmd;
        double brP = vyR + vxR - omegaCmd;

        double max = Math.max(1.0, Math.max(Math.abs(flP),
                Math.max(Math.abs(frP), Math.max(Math.abs(blP), Math.abs(brP)))));
        fl.setPower(flP / max);
        bl.setPower(blP / max);
        fr.setPower(frP / max);
        br.setPower(brP / max);
    }

    private void stepIndexRowMajor() {
        if (++j >= HOOD_LIST.length) { j = 0; i = Math.min(i + 1, RPM_LIST.length - 1); }
    }

    private static int clampIndex(int v, int n) {
        return Range.clip(v, 0, n - 1);
    }

    private static double deadband(double v) { return Math.abs(v) > ShotTuningConstants.STICK_DB ? v : 0.0; }
    private static double clamp(double v) { return Math.max(-1.2, Math.min(ShotTuningConstants.OMEGA_MAX, v)); }

    private double batteryV() {
        double best = 0.0;
        for (VoltageSensor vs : hardwareMap.voltageSensor) {
            double v = vs.getVoltage();
            if (v > best) best = v;
        }
        return best;
    }

    @SuppressLint("DefaultLocale")
    private void addTelemetry(double rpmTarget) {
        telemetry.addLine("--- SHOT TUNING ---");
        telemetry.addData("grid", "i=%d/%d  j=%d/%d", i, RPM_LIST.length - 1, j, HOOD_LIST.length - 1);
        telemetry.addData("lock", aimLock ? "ON" : "OFF");

        Pose2D pose = state.getPose();
        double x = pose.getX(DistanceUnit.METER);
        double y = pose.getY(DistanceUnit.METER);
        double thDeg = Math.toDegrees(pose.getHeading(AngleUnit.RADIANS));

        double tps = hw.getShooterMotor().getVelocity();
        double rpmM = tps * 60.0 / TPR_MOTOR;

        telemetry.addData("set",  "rpm_t=%.0f  hood=%.3f", rpmTarget, HOOD_LIST[j]);
        telemetry.addData("meas", "rpm_m=%.0f  V=%.2f", rpmM, batteryV());
        telemetry.addData("pose", "(%.3f, %.3f) m  θ=%.1f°", x, y, thDeg);
        telemetry.addData("LL",   "seen=%b  tx=%.1f°  pipe=%d", llValid, llValid ? llTxDeg : Double.NaN, LIMELIGHT_PIPELINE);

        telemetry.addLine("#  i  j  rpm_t  rpm_m  hood   x     y    θdeg  ψerr°  seen  Vv");
        int start = Math.max(0, log.size() - TELEMETRY_ROWS);
        for (int k = start; k < log.size(); k++) {
            Row r = log.get(k);
            telemetry.addLine(String.format(Locale.US,
                    "%2d %2d %2d %5.0f %5.0f  %.3f  %.2f  %.2f  %6.1f  %6.1f  %4s  %.2f",
                    k, r.i, r.j, r.rpm_t, r.rpm_m, r.hood, r.x, r.y, r.th_deg,
                    (Double.isNaN(r.psi_deg) ? 999.0 : r.psi_deg),
                    r.seen ? "Y" : "N", r.V));
        }
        telemetry.addLine("A=log  X=undo  Y=skip  RB=toggle lock  DPad=select  LS/RS=drive");
        telemetry.update();
    }

    // simple rising-edge helper
    private static class Edge {
        boolean last = false;
        boolean press(boolean now) { boolean r = now && !last; last = now; return r; }
    }
}
