//package org.firstinspires.ftc.teamcode.tuning;
//
//import static org.firstinspires.ftc.teamcode.localisation.Constants.*;
//import static org.firstinspires.ftc.teamcode.subsystems.ShooterConstants.*;
//import static org.firstinspires.ftc.teamcode.tuning.ShooterTuningConstants.*;
//
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//import org.firstinspires.ftc.teamcode.RobotHardware;
//import org.firstinspires.ftc.teamcode.localisation.StateEstimator;
//import org.firstinspires.ftc.teamcode.subsystems.Shooter;
//import org.firstinspires.ftc.teamcode.vision.AprilTagLocalizerLimelight;
//
//import java.util.Locale;
//
//@TeleOp(name = "ShooterTuning", group = "Tuning")
//public class ShooterTuning extends LinearOpMode {
//    private RobotHardware hw;
//    private StateEstimator state;
//    private AprilTagLocalizerLimelight ll;
//    private Shooter shooter;
//    private DcMotorEx shooterMotor;
//    private Servo hood;
//    private VoltageSensor vbat;
//    private int sel = 0;
//    private enum SpinPhase { READY, RAMPING, SETTLING, HOLD }
//    private static final class Edge { boolean last; boolean press(boolean now){ boolean p = now && !last; last = now; return p; } }
//    private final Edge aE=new Edge(), bE=new Edge(), xE=new Edge(), yE=new Edge(),
//            lbE=new Edge(), rbE=new Edge(), leftE=new Edge(), rightE=new Edge();
//
//    interface Tuner {
//        String name();
//        void onSelect();
//        void onStart();
//        void onLoop();
//    }
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        hw = new RobotHardware(this);
//        hw.initPinpoint();
//        hw.initLimeLight(100);
//        hw.setLimelightPipeline(0);
//        hw.initShooter();
//        hw.initHood();
//
//        shooterMotor = hw.getShooterMotor();
//        hood = hw.getHoodServo();
//        vbat = hardwareMap.voltageSensor.iterator().hasNext()
//                ? hardwareMap.voltageSensor.iterator().next() : null;
//
//        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        shooterMotor.setVelocityPIDFCoefficients(P, I, D, F);
//
//        ll = new AprilTagLocalizerLimelight(hw.getLimelight());
//        state = new StateEstimator(this, hw.getPinpoint(), ll);
//
//        shooter = new Shooter(shooterMotor, /*map*/ null, this);
//
//        Tuner[] tuners = new Tuner[]{
//                new SpinupTuner(),
//                new ShotLUTPointTuner(),
//                new VoltageFFTuner(),
//                new EstimatorSnapTuner()
//        };
//
//        while (!isStarted() && !isStopRequested()) {
//            if (rbE.press(gamepad1.right_bumper) || rightE.press(gamepad1.dpad_right)) sel = (sel+1) % tuners.length;
//            if (lbE.press(gamepad1.left_bumper) || leftE.press(gamepad1.dpad_left)) sel = (sel-1+ tuners.length)% tuners.length;
//
//            telemetry.clear();
//            telemetry.addLine("--- Assist Tuning ---");
//            telemetry.addData("Select", "%d/%d: %s", sel+1, tuners.length, tuners[sel].name());
//            telemetry.addLine("LB/RB or DPad ←/→ to change. START to run.");
//            telemetry.update();
//            idle();
//        }
//        if (isStopRequested()) return;
//
//        tuners[sel].onSelect();
//        tuners[sel].onStart();
//
//        while (opModeIsActive()) {
//            state.update();
//            tuners[sel].onLoop();
//            telemetry.update();
//            idle();
//        }
//
//        shooterMotor.setPower(0);
//        hw.closeVision();
//    }
//
//    // === Tuner 1: Shooter spin-up timing ===
//    private final class SpinupTuner implements Tuner {
//        private double targetRpm = DEFAULT_RPM;
//        private double hoodPos = DEFAULT_HOOD;
//
//        private SpinPhase phase = SpinPhase.READY;
//        private long tStart, tInBand, tHold, spinMs = -1;
//
//        @Override public String name(){ return "Shooter Spin-up"; }
//
//        @Override public void onSelect() {
//            telemetry.clear();
//            telemetry.addLine("X/RT: ramp A:+RPM Y:-RPM LB/RB: hood ± B: reset");
//        }
//
//        @Override public void onStart() {
//            hood.setPosition(hoodPos);
//            shooter.setRpm(IDLE_SCALE * targetRpm);
//            phase = SpinPhase.READY;
//            spinMs = -1;
//        }
//
//        @Override public void onLoop() {
//            if (aE.press(gamepad1.a)) targetRpm += RPM_STEP;
//            if (yE.press(gamepad1.y)) targetRpm = Math.max(0, targetRpm - RPM_STEP);
//            if (lbE.press(gamepad1.left_bumper)) { hoodPos = clamp01(hoodPos - HOOD_STEP); hood.setPosition(hoodPos); }
//            if (rbE.press(gamepad1.right_bumper)) { hoodPos = clamp01(hoodPos + HOOD_STEP); hood.setPosition(hoodPos); }
//            if (bE.press(gamepad1.b)) { onStart(); }
//
//            boolean fire = gamepad1.right_trigger > TRIGGER_DB || xE.press(gamepad1.x);
//
//            double tps = shooterMotor.getVelocity();
//            double rpmNow = tpsToRpmOut(tps);
//            double motorRpm = tpsToRpmMotor(tps);
//            double vb = (vbat != null) ? vbat.getVoltage() : Double.NaN;
//
//            switch (phase){
//                case READY:
//                    shooter.setRpm(IDLE_SCALE * targetRpm);
//                    if (fire) {
//                        shooter.setRpm(targetRpm);
//                        tStart = now();
//                        tInBand = 0;
//                        spinMs = -1;
//                        phase = SpinPhase.RAMPING;
//                    }
//                    break;
//                case RAMPING:
//                    if (inBand(rpmNow, targetRpm, RPM_BAND)) {
//                        if (tInBand == 0) tInBand = now();
//                        if (now() - tInBand >= SETTLE_MS) {
//                            spinMs = now() - tStart;
//                            tHold = now();
//                            phase = SpinPhase.SETTLING;
//                        }
//                    } else tInBand = 0;
//                    break;
//                case SETTLING:
//                    if (now() - tHold >= HOLD_MS) phase = SpinPhase.HOLD;
//                    break;
//                case HOLD:
//                    break;
//            }
//
//            telemetry.clear();
//            telemetry.addLine("[Spin-up]");
//            telemetry.addData("Target RPM", fmt(targetRpm));
//            telemetry.addData("Idle RPM", fmt(IDLE_SCALE * targetRpm));
//            telemetry.addData("Hood pos", fmt(hoodPos));
//            telemetry.addData("RPM now", fmt(rpmNow));
//            telemetry.addData("Motor RPM", fmt(motorRpm));
//            telemetry.addData("Err RPM", fmt(targetRpm - rpmNow));
//            telemetry.addData("Spin-up [ms]", (spinMs<0) ? "…" : String.valueOf(spinMs));
//            telemetry.addData("Battery [V]", fmt(vb));
//            telemetry.addData("Phase", phase);
//        }
//    }
//
//    // === Tuner 2: Shot LUT point ===
//    private final class ShotLUTPointTuner implements Tuner {
//        private double rpm = DEFAULT_RPM;
//        private double hoodPos = DEFAULT_HOOD;
//        private boolean scored = false;
//
//        @Override public String name(){ return "Shot LUT Point"; }
//
//        @Override public void onSelect() {
//            telemetry.clear();
//            telemetry.addLine("LB/RB: hood ± A:+RPM Y:-RPM X:fire B:toggle scored");
//        }
//
//        @Override public void onStart() {
//            hood.setPosition(hoodPos);
//            shooter.setRpm(IDLE_SCALE * rpm);
//        }
//
//        @Override public void onLoop() {
//            if (lbE.press(gamepad1.left_bumper)) { hoodPos = clamp01(hoodPos - HOOD_STEP); hood.setPosition(hoodPos); }
//            if (rbE.press(gamepad1.right_bumper)) { hoodPos = clamp01(hoodPos + HOOD_STEP); hood.setPosition(hoodPos); }
//            if (aE.press(gamepad1.a)) rpm += RPM_STEP;
//            if (yE.press(gamepad1.y)) rpm = Math.max(0, rpm - RPM_STEP);
//            if (xE.press(gamepad1.x)) shooter.setRpm(rpm);
//            if (bE.press(gamepad1.b)) scored = !scored;
//
//            LLResult res = ll.getResult();
//            Pose3D mt2 = (res != null && res.isValid()) ? res.getBotpose_MT2() : null;
//
//            double tps = shooterMotor.getVelocity();
//            double rpmNow = tpsToRpmOut(tps);
//
//            telemetry.clear();
//            telemetry.addLine("[LUT Point]");
//            telemetry.addData("RPM target", fmt(rpm));
//            telemetry.addData("RPM now", fmt(rpmNow));
//            telemetry.addData("Err RPM", fmt(rpm - rpmNow));
//            telemetry.addData("Hood pos", fmt(hoodPos));
//            telemetry.addData("Scored", scored);
//
//            if (mt2 != null) {
//                telemetry.addData("LL pose x,y [m]", "(%.3f, %.3f)", mt2.getPosition().x, mt2.getPosition().y);
//                telemetry.addData("LL yaw [deg]", fmt(Math.toDegrees(mt2.getOrientation().getYaw(AngleUnit.RADIANS))));
//            } else telemetry.addLine("Tag: not visible");
//
//            telemetry.addData("Robot (odom) x,y [m]", "(%.3f, %.3f)",
//                    state.getPose().getX(DistanceUnit.METER), state.getPose().getY(DistanceUnit.METER));
//        }
//    }
//
//    // === Tuner 3: Voltage FF display ===
//    private final class VoltageFFTuner implements Tuner {
//        private double rpm = DEFAULT_RPM;
//        private double vNom = V_NOM;
//
//        @Override public String name(){ return "Voltage FF Check"; }
//
//        @Override public void onSelect() {
//            telemetry.clear();
//            telemetry.addLine("A:+RPM Y:-RPM LB/RB: Vnom ±0.1 (display-only F suggestion)");
//        }
//
//        @Override public void onStart() {
//            shooterMotor.setVelocityPIDFCoefficients(P, I, D, F);
//            shooter.setRpm(rpm);
//        }
//
//        @Override public void onLoop() {
//            if (aE.press(gamepad1.a)) rpm += RPM_STEP;
//            if (yE.press(gamepad1.y)) rpm = Math.max(0, rpm - RPM_STEP);
//            if (lbE.press(gamepad1.left_bumper))  vNom -= 0.1;
//            if (rbE.press(gamepad1.right_bumper)) vNom += 0.1;
//
//            shooter.setRpm(rpm);
//
//            double vb = (vbat != null) ? vbat.getVoltage() : Double.NaN;
//            double tps = shooterMotor.getVelocity();
//            double rpmNow = tpsToRpmOut(tps);
//            double err = rpm - rpmNow;
//
//            Double fScaled = (Double.isNaN(vb) || vb<=0) ? null : F * (vNom / vb);
//
//            telemetry.clear();
//            telemetry.addLine("[Voltage FF]");
//            telemetry.addData("Target RPM", fmt(rpm));
//            telemetry.addData("RPM now", fmt(rpmNow));
//            telemetry.addData("Err RPM", fmt(err));
//            telemetry.addData("Battery [V]", fmt(vb));
//            telemetry.addData("Vnom [V]", fmt(vNom));
//            telemetry.addData("F (const)", fmt(F));
//            telemetry.addData("F_scaled", (fScaled==null) ? "NaN" : String.format(Locale.US,"%.5f", fScaled));
//        }
//    }
//
//    // === Tuner 4: Estimator snap check ===
//    private final class EstimatorSnapTuner implements Tuner {
//        private boolean snapped = false;
//
//        @Override public String name(){ return "Estimator Snap Check"; }
//
//        @Override public void onSelect() {
//            telemetry.clear();
//            telemetry.addLine("X: snap Pinpoint to LL pose when visible and stationary.");
//        }
//
//        @Override public void onStart() {}
//
//        @Override public void onLoop() {
//            LLResult r = ll.getResult();
//            Pose3D mt2 = (r != null && r.isValid()) ? r.getBotpose_MT2() : null;
//
//            double ox = state.getPose().getX(DistanceUnit.METER);
//            double oy = state.getPose().getY(DistanceUnit.METER);
//            double oh = state.getHeading();
//
//            double llx = Double.NaN, lly = Double.NaN, llh = Double.NaN;
//            double dx = Double.NaN, dy = Double.NaN, dth = Double.NaN;
//
//            if (mt2 != null) {
//                llx = mt2.getPosition().x;
//                lly = mt2.getPosition().y;
//                llh = mt2.getOrientation().getYaw(AngleUnit.RADIANS);
//                dx = llx - ox; dy = lly - oy; dth = wrap(llh - oh);
//
//                boolean stationary = Math.hypot(
//                        state.getChassisSpeedsField().vxMetersPerSecond,
//                        state.getChassisSpeedsField().vyMetersPerSecond
//                ) < STATIONARY_V_MPS &&
//                        Math.abs(state.getChassisSpeedsField().omegaRadiansPerSecond) < STATIONARY_OMEGA_RAD;
//
//                if (xE.press(gamepad1.x) && stationary) {
//                    hw.getPinpoint().setPosition(new Pose2D(
//                            DistanceUnit.METER, llx, lly, AngleUnit.RADIANS, llh
//                    ));
//                    snapped = true;
//                }
//            }
//
//            telemetry.clear();
//            telemetry.addLine("[Estimator]");
//            telemetry.addData("Pinpoint x,y,θ", "(%.3f, %.3f) | %.1f°", ox, oy, Math.toDegrees(oh));
//            telemetry.addData("LL seen", (mt2 != null));
//            telemetry.addData("LL x,y,θ", "(%.3f, %.3f) | %.1f°", llx, lly, Math.toDegrees(llh));
//            telemetry.addData("Residual dx,dy,dθ", "(%.3f, %.3f) | %.1f°", dx, dy, Math.toDegrees(dth));
//            telemetry.addData("Snapped", snapped);
//        }
//    }
//
//    // Helpers
//    private static boolean inBand(double v, double tgt, double band){ return Math.abs(v - tgt) <= band; }
//    private static double tpsToRpmOut(double tps){ return tps * 60.0 / TPR_OUTPUT; }
//    private static double tpsToRpmMotor(double tps){ return tps * 60.0 / TPR_MOTOR; }
//    private static String fmt(double v){ return String.format(Locale.US,"%.2f", v); }
//    private static long now(){ return System.currentTimeMillis(); }
//    private static double wrap(double a){ while(a>Math.PI)a-=2*Math.PI; while(a<-Math.PI)a+=2*Math.PI; return a; }
//    private static double clamp01(double v){ return Math.max(0, Math.min(1, v)); }
//}
