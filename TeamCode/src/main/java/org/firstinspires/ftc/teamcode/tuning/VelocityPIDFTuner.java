package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp(name = "VelocityPIDF Tuner", group = "Tuning")
public class VelocityPIDFTuner extends LinearOpMode {
    // Made by ChatGPT may or may not work

    // === USER SETTINGS ===
    private static final String MOTOR_NAME = "shooter";          // rename to your config name
    private static final double TICKS_PER_REV = 384.5397923875; // output shaft TPR for your gearbox

    // Characterization steps (duty cycles). You can change these if needed.
    private static final double[] POWERS = {0.20, 0.40, 0.60, 0.80, 1.00};
    private static final long WARMUP_MS = 800;   // time to reach steady state before sampling
    private static final long SAMPLE_MS = 1200;  // averaging window per step

    // PIDF increments
    private static final double STEP_FINE = 0.0001;
    private static final double STEP_MED  = 0.0010;
    private static final double STEP_COARSE = 0.0100;

    // Runtime state
    private DcMotorEx motor;

    private static class Edge {
        boolean last;
        boolean press(boolean now) { boolean p = now && !last; last = now; return p; }
    }

    private static class Fit {
        double slopeA;     // ticks/sec per 1.0 power (through-origin slope)
        double tpsMax;     // observed max ticks/sec at power=1
        double F;          // feedforward gain ~= 1/a
    }

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Ensure no built-in velocity control during characterization
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Cache and show starting PIDF
        DcMotorEx PIDMotor = motor; // alias for readability
        double P = 0, I = 0, D = 0, F = 0; // will fill after characterization

        // Controller edges
        Edge aEdge = new Edge();
        Edge yEdge = new Edge();
        Edge upEdge = new Edge();
        Edge dnEdge = new Edge();
        Edge lfEdge = new Edge();
        Edge rtEdge = new Edge();
        Edge xEdge = new Edge();
        Edge bEdge = new Edge();

        Fit fit = null;
        double maxTpsEstimate = 0; // used for target scaling after fit

        waitForStart();

        // Switch to encoder mode once we actually start teleop control; we'll switch back to WITHOUT during characterization.
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()) {
            // === Controls ===
            boolean a = gamepad1.a; // run characterization
            boolean y = gamepad1.y; // apply F to controller
            boolean up = gamepad1.dpad_up;
            boolean dn = gamepad1.dpad_down;
            boolean lf = gamepad1.dpad_left;
            boolean rt = gamepad1.dpad_right;
            boolean x  = gamepad1.x; // increase I
            boolean b  = gamepad1.b; // decrease I
            boolean lb = gamepad1.left_bumper;
            boolean rb = gamepad1.right_bumper;

            double inc = lb ? STEP_FINE : (rb ? STEP_COARSE : STEP_MED);

            if (aEdge.press(a)) {
                fit = characterize();
                if (fit != null) {
                    F = fit.F;
                    maxTpsEstimate = fit.tpsMax; // conservative initial max
                }
            }

            if (yEdge.press(y) && fit != null) {
                // Apply current PIDF using built-in controller
                setRunUsingEncoderIfNeeded();
                PIDMotor.setVelocityPIDFCoefficients(P, I, D, F);
            }

            if (upEdge.press(up)) { P += inc; setPidIfEncoder(PIDMotor, P, I, D, F); }
            if (dnEdge.press(dn)) { P = Math.max(0, P - inc); setPidIfEncoder(PIDMotor, P, I, D, F); }
            if (lfEdge.press(lf)) { D = Math.max(0, D - inc); setPidIfEncoder(PIDMotor, P, I, D, F); }
            if (rtEdge.press(rt)) { D += inc; setPidIfEncoder(PIDMotor, P, I, D, F); }
            if (xEdge.press(x))   { I += inc; setPidIfEncoder(PIDMotor, P, I, D, F); }
            if (bEdge.press(b))   { I = Math.max(0, I - inc); setPidIfEncoder(PIDMotor, P, I, D, F); }

            // Target set via triggers: RT forward, LT reverse
            double targetScale = (fit != null && maxTpsEstimate > 0) ? maxTpsEstimate : 1.0;
            double tgtTps = (gamepad1.right_trigger - gamepad1.left_trigger) * targetScale;

            // Command velocity in ticks/sec using built-in controller
            setRunUsingEncoderIfNeeded();
            motor.setVelocity(tgtTps);

            // Telemetry
            double tps = motor.getVelocity();
            double rpmOut = tps * 60.0 / TICKS_PER_REV;
            double err = tgtTps - tps;

            telemetry.addData("tps", format(tps));
            telemetry.addData("rpm", format(rpmOut));
            telemetry.addData("target_tps", format(tgtTps));
            telemetry.addData("err_tps", format(err));
            telemetry.addLine();

            if (fit != null) {
                telemetry.addData("F (from fit)", format(fit.F));
                telemetry.addData("a slope (tps/power)", format(fit.slopeA));
                telemetry.addData("tps_max obs", format(fit.tpsMax));
            } else {
                telemetry.addLine("Press A to characterize feedforward (F)");
            }

            telemetry.addLine();
            telemetry.addData("P", format(P));
            telemetry.addData("I", format(I));
            telemetry.addData("D", format(D));
            telemetry.addData("F", format(F));
            telemetry.addLine("LB=fine, RB=coarse | DPad Up/Down=P, Left/Right=D, X/B=I | Y=apply PIDF | RT/LT=set target");
            telemetry.update();
        }

        // Stop motor when OpMode ends
        motor.setPower(0);
    }

    private void setRunUsingEncoderIfNeeded() {
        if (motor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void setRunWithoutEncoderIfNeeded() {
        if (motor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void setPidIfEncoder(DcMotorEx m, double P, double I, double D, double F) {
        if (motor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            m.setVelocityPIDFCoefficients(P, I, D, F);
        }
    }

    private Fit characterize() throws InterruptedException {
        // Characterize motor open-loop: measure ticks/sec at a few power levels, fit slope through origin.
        setRunWithoutEncoderIfNeeded();

        double sumPP = 0.0;     // sum(power^2)
        double sumPT = 0.0;     // sum(power * ticks/sec)
        double maxTps = 0.0;

        for (double p : POWERS) {
            double tps = measureTpsAtPower(p);
            sumPP += p * p;
            sumPT += p * tps;
            if (tps > maxTps) maxTps = tps;
        }

        if (sumPP <= 1e-9) return null;

        Fit f = new Fit();
        f.slopeA = sumPT / sumPP;  // ticks/sec per 1.0 power
        f.tpsMax = maxTps;
        f.F = 1.0 / f.slopeA;      // feedforward gain for built-in controller (power ≈ F * target_tps)

        return f;
    }

    private double measureTpsAtPower(double power) throws InterruptedException {
        motor.setPower(power);
        sleep(WARMUP_MS);

        long start = System.nanoTime();
        long end;
        double sum = 0.0;
        int n = 0;
        do {
            sum += motor.getVelocity(); // ticks/sec
            n++;
            sleep(10); // 100 Hz sampling is plenty
            end = System.nanoTime();
        } while (((end - start) / 1e6) < SAMPLE_MS && opModeIsActive());

        return n > 0 ? sum / n : 0.0;
    }

    private static String format(double v) {
        return String.format(java.util.Locale.US, "%.3f", v);
    }
}
