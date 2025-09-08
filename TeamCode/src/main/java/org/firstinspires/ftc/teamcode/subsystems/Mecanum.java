package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.localisation.StateEstimator;

@Configurable
public class Mecanum {
    OpMode opmode;

    ElapsedTime timer = new ElapsedTime();

    // Motors
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // State estimation
    private final StateEstimator state;

    // Motor constants
    private double prevFrontLeftPower, prevBackLeftPower, prevFrontRightPower, prevBackRightPower;
    public static double SLOW_MODE_FACTOR = 0.5;
    public static double CACHING_THRESHOLD = 0.005;

    // Angle and slow mode modifiers
    private boolean angleLock = false, slowMode = false, fieldCentricEnabled = false;
    private double targetHeading = 0;
    private boolean prevAToggle = false;
    private boolean prevBToggle = false;
    private boolean prevXToggle = false;

    // PD
    public static double KP = 0.01;
    public static double KD = 0;

    // Deadbands & clamps
    public static double STICK_DEAD_BAND = 0.05;
    public static double ROTATE_DEAD_BAND = 0.08;
    public static double OMEGA_MAX = 1.0;

    // Constructor
    public Mecanum(DcMotorEx frontLeft, DcMotorEx backLeft, DcMotorEx frontRight, DcMotorEx backRight, StateEstimator state, OpMode opmode) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.state = state;
        this.opmode = opmode;
    }

    public void operateMecanum() {
        handleToggles();
        if (fieldCentricEnabled) {
            fieldCentric();
        } else {
            robotCentric();
        }
        addTelemetry();
    }

    public void addTelemetry() {
        Pose2D currentPos = state.getPose();
        ChassisSpeeds currentSpeeds = state.getChassisSpeedsRobot();

        opmode.telemetry.addData("Heading (d):", currentPos.getHeading(AngleUnit.DEGREES));
        opmode.telemetry.addData("X Position (m):", currentPos.getX(DistanceUnit.METER));
        opmode.telemetry.addData("Y Position (m):", currentPos.getY(DistanceUnit.METER));
        opmode.telemetry.addData("Angular Velocity", currentSpeeds.omegaRadiansPerSecond);
        opmode.telemetry.addData("X Velocity:", currentSpeeds.vxMetersPerSecond);
        opmode.telemetry.addData("Y Velocity:", currentSpeeds.vyMetersPerSecond);
        opmode.telemetry.addData("FL Power:", frontLeft.getPower());
        opmode.telemetry.addData("BL Power:", backLeft.getPower());
        opmode.telemetry.addData("FR Power:", frontRight.getPower());
        opmode.telemetry.addData("BR Power:", backRight.getPower());
    }

    private void handleToggles() {
        boolean aNow = opmode.gamepad1.a;
        if (aNow && !prevAToggle) {
            angleLock = !angleLock;
            if (angleLock) { targetHeading = state.getHeading(); }
        }
        prevAToggle = aNow;

        boolean bNow = opmode.gamepad1.b;
        if (bNow && !prevBToggle) { slowMode = !slowMode; }
        prevBToggle = bNow;

        boolean xNow = opmode.gamepad1.x;
        if (xNow && !prevXToggle) { fieldCentricEnabled = !fieldCentricEnabled; }
        prevXToggle = xNow;
    }

    private void applyRobotVel(double vx, double vy, double omega) {
        if(slowMode) {
            vx *= SLOW_MODE_FACTOR;
            vy *= SLOW_MODE_FACTOR;
            omega *= SLOW_MODE_FACTOR;
        }

        double x = -vy;
        double y = vx;

        double frontLeftPower = y + x + omega;
        double backLeftPower = y - x + omega;
        double frontRightPower = y - x - omega;
        double backRightPower = y + x - omega;

        double max = Math.max(1.0, Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)))));

        frontLeftPower /= max;
        backLeftPower /= max;
        frontRightPower /= max;
        backRightPower /= max;

        if (Math.abs(frontLeftPower - prevFrontLeftPower) >= CACHING_THRESHOLD) { frontLeft.setPower(frontLeftPower); }
        if (Math.abs(backLeftPower - prevBackLeftPower) >= CACHING_THRESHOLD) { backLeft.setPower(backLeftPower); }
        if (Math.abs(frontRightPower - prevFrontRightPower) >= CACHING_THRESHOLD) { frontRight.setPower(frontRightPower); }
        if (Math.abs(backRightPower - prevBackRightPower) >= CACHING_THRESHOLD) { backRight.setPower(backRightPower); }

        prevFrontLeftPower = frontLeftPower;
        prevBackLeftPower = backLeftPower;
        prevFrontRightPower = frontRightPower;
        prevBackRightPower  = backRightPower;
    }

    private void robotCentric() {
        ChassisSpeeds currentChassisSpeeds = state.getChassisSpeedsRobot();
        double vx = deadband(-opmode.gamepad1.left_stick_y, STICK_DEAD_BAND);
        double vy = deadband(opmode.gamepad1.left_stick_x, STICK_DEAD_BAND);
        double rotateStick = opmode.gamepad1.right_stick_x;

        double omegaCmd;
        boolean driverRotating = Math.abs(rotateStick) > ROTATE_DEAD_BAND;

        if (driverRotating) {
            omegaCmd = rotateStick;
            targetHeading = state.getHeading();
        } else if (angleLock) {
            double heading = state.getHeading();
            double error = wrapRad(targetHeading - heading);

            double gyroRate = currentChassisSpeeds.omegaRadiansPerSecond;

            omegaCmd = KP * error - KD * gyroRate;
            omegaCmd = clamp(omegaCmd, -OMEGA_MAX, OMEGA_MAX);
        } else {
            omegaCmd = 0;
        }

        applyRobotVel(vx, vy, omegaCmd);
    }

    private void fieldCentric() {
        ChassisSpeeds currentChassisSpeeds = state.getChassisSpeedsField();
        double vxF = deadband(-opmode.gamepad1.left_stick_y, STICK_DEAD_BAND);
        double vyF = deadband(opmode.gamepad1.left_stick_x, STICK_DEAD_BAND);
        double h = state.getHeading();
        double cos = Math.cos(h), sin = Math.sin(h);

        double vxR = vxF * cos + vyF * sin;
        double vyR = -vxF * sin + vyF * cos;

        double rotateStick = opmode.gamepad1.right_stick_x;

        double omegaCmd;
        if (Math.abs(rotateStick) > ROTATE_DEAD_BAND) {
            omegaCmd = rotateStick;
            targetHeading = state.getHeading();
        } else if (angleLock) {
            double gyroRate = state.getChassisSpeedsRobot().omegaRadiansPerSecond;
            double err = wrapRad(targetHeading - state.getHeading());
            omegaCmd = clamp(KP * err - KD * gyroRate, -OMEGA_MAX, OMEGA_MAX);
        } else {
            omegaCmd = 0.0;
        }

        applyRobotVel(vxR, vyR, omegaCmd);
    }

    private static double deadband(double v, double d) { return Math.abs(v) > d ? v : 0.0; }
    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
    private static double wrapRad(double a) {
        while (a > Math.PI) a -= 2*Math.PI;
        while (a < -Math.PI) a += 2*Math.PI;
        return a;
    }
}
