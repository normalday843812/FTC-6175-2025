package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.Drivetrain;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.ArrayList;
import java.util.List;

public class PedroMecanumDrivetrain extends Drivetrain {
    private final DcMotorEx fl, fr, bl, br;
    private final List<VoltageSensor> voltageSensors = new ArrayList<>();

    private double xVel = 0.0, yVel = 0.0;
    private boolean teleopBrake = false;

    public PedroMecanumDrivetrain(HardwareMap hw,
                                  String frontLeft, String frontRight,
                                  String backLeft,  String backRight) {
        fl = hw.get(DcMotorEx.class, frontLeft);
        fr = hw.get(DcMotorEx.class, frontRight);
        bl = hw.get(DcMotorEx.class, backLeft);
        br = hw.get(DcMotorEx.class, backRight);

        for (VoltageSensor vs : hw.voltageSensor) voltageSensors.add(vs);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // If your right side should be reversed, uncomment:
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public double[] calculateDrive(Vector correctivePower,
                                   Vector headingPower,
                                   Vector pathingPower,
                                   double robotHeading) {
        // Combine field-centric translational vectors
        double fx = clamp(correctivePower.getXComponent() + pathingPower.getXComponent());
        double fy = clamp(correctivePower.getYComponent() + pathingPower.getYComponent());

        // Field -> robot transform using current heading
        double cos = Math.cos(-robotHeading);
        double sin = Math.sin(-robotHeading);
        double xr =  fx * cos - fy * sin;  // +x right
        double yr =  fx * sin + fy * cos;  // +y forward

        // Turn command from heading vector. If turning is inverted, negate this.
        double w  = clamp(headingPower.getXComponent());
        // Alternative if you prefer: double w = clamp(headingPower.getYComponent());

        // Mecanum mix (robot-centric)
        double flPow = yr + xr + w;
        double frPow = yr - xr - w;
        double blPow = yr - xr + w;
        double brPow = yr + xr - w;

        // Normalize
        double max = Math.max(1.0, Math.max(Math.abs(flPow),
                Math.max(Math.abs(frPow), Math.max(Math.abs(blPow), Math.abs(brPow)))));
        flPow /= max; frPow /= max; blPow /= max; brPow /= max;

        // Optional voltage compensation (uses base-class toggles/nominalV)
        if (isVoltageCompensation()) {
            double v = getVoltage();
            if (v > 1e-3) {
                double scale = getNominalVoltage() / v;
                flPow *= scale; frPow *= scale; blPow *= scale; brPow *= scale;
            }
        }

        // Respect maxPowerScaling from base class
        double s = getMaxPowerScaling();
        flPow *= s; frPow *= s; blPow *= s; brPow *= s;

        return new double[]{ flPow, frPow, blPow, brPow };
    }

    @Override
    public void runDrive(double[] p) {
        fl.setPower(p[0]); fr.setPower(p[1]); bl.setPower(p[2]); br.setPower(p[3]);
    }

    @Override
    public void breakFollowing() {
        fl.setPower(0); fr.setPower(0); bl.setPower(0); br.setPower(0);
    }

    @Override
    public void startTeleopDrive() {
        startTeleopDrive(true);
    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        teleopBrake = brakeMode;
        DcMotor.ZeroPowerBehavior z = brakeMode ? DcMotor.ZeroPowerBehavior.BRAKE
                : DcMotor.ZeroPowerBehavior.FLOAT;
        fl.setZeroPowerBehavior(z);
        fr.setZeroPowerBehavior(z);
        bl.setZeroPowerBehavior(z);
        br.setZeroPowerBehavior(z);
    }

    @Override
    public void setXVelocity(double xMovement) { xVel = xMovement; }

    @Override
    public void setYVelocity(double yMovement) { yVel = yMovement; }

    @Override
    public double xVelocity() { return xVel; }

    @Override
    public double yVelocity() { return yVel; }

    @Override
    public double getVoltage() {
        double best = Double.MAX_VALUE;
        for (VoltageSensor vs : voltageSensors) {
            double v = vs.getVoltage();
            if (v > 0 && v < best) best = v;
        }
        return best == Double.MAX_VALUE ? 12.0 : best;
    }

    @Override
    public String debugString() {
        return String.format("TeleOp v(x,y)=(%.2f, %.2f) brake=%s", xVel, yVel, teleopBrake);
    }

    @Override
    public void updateConstants() {
        // no-op; add if you expose tunables
    }

    private static double clamp(double v) { return Math.max(-1.0, Math.min(1.0, v)); }
}
