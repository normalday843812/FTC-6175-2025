package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static org.firstinspires.ftc.teamcode.subsystems.MecanumConstants.*;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.localisation.StateEstimatorTwoWheel;

@Configurable
public class MecanumTwoOdom {
    OpMode opmode;

    // Motors
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // State estimation
    private final StateEstimatorTwoWheel state;

    // Controls
    private final GamepadMap map;

    // Angle and slow mode modifiers
    private boolean angleLock = false, slowMode = false, fieldCentricEnabled = false;
    private double targetHeading = 0;
    private boolean prevAToggle = false;
    private boolean prevBToggle = false;
    private boolean prevXToggle = false;

    // Constructor
    public MecanumTwoOdom(DcMotorEx frontLeft, DcMotorEx backLeft, DcMotorEx frontRight, DcMotorEx backRight, StateEstimatorTwoWheel state, GamepadMap map, OpMode opmode) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.state = state;
        this.map = map;
        this.opmode = opmode;
    }

    public void operateMecanum() {
        handleToggles();
        if (map.resetPinpointButton) {
            resetFieldCentric();
        }
        if (fieldCentricEnabled) {
            fieldCentric();
        } else {
            robotCentric();
        }
        if (TELEMETRY_ENABLED) {
            addTelemetry();
        }
    }

    private void addTelemetry() {
        opmode.telemetry.addLine("--- MECANUM ---");
        opmode.telemetry.addData("FL Power:", frontLeft.getPower());
        opmode.telemetry.addData("BL Power:", backLeft.getPower());
        opmode.telemetry.addData("FR Power:", frontRight.getPower());
        opmode.telemetry.addData("BR Power:", backRight.getPower());
        opmode.telemetry.addData("Control:", fieldCentricEnabled ? "Field Centric" : "Robot Centric");
        opmode.telemetry.addData("Slow Mode:", slowMode);
        opmode.telemetry.addData("Angle Lock:", angleLock);
        opmode.telemetry.addData("Target Heading:", Math.toDegrees(targetHeading) + "°");
    }

    private void handleToggles() {
        boolean aNow = map.angleLockToggle;
        if (aNow && !prevAToggle) {
            angleLock = !angleLock;
            if (angleLock) { targetHeading = state.getHeading(); }
        }
        prevAToggle = aNow;

        boolean bNow = map.slowModeToggle;
        if (bNow && !prevBToggle) { slowMode = !slowMode; }
        prevBToggle = bNow;

        boolean xNow = map.fieldCentricToggle;
        if (xNow && !prevXToggle) { fieldCentricEnabled = !fieldCentricEnabled; }
        prevXToggle = xNow;
    }

    private void applyRobotVel(double vx, double vy, double omega) {
        if(slowMode) {
            vx *= SLOW_MODE_FACTOR;
            vy *= SLOW_MODE_FACTOR;
            omega *= SLOW_MODE_FACTOR;
        }

        double x = vy;
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

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    private void robotCentric() {
        double vx = deadband(map.forward, STICK_DEAD_BAND);
        double vy = deadband(map.strafe, STICK_DEAD_BAND);
        double rotateStick = map.rotate;

        double omegaCmd;
        boolean driverRotating = Math.abs(rotateStick) > ROTATE_DEAD_BAND;

        if (driverRotating) {
            omegaCmd = rotateStick;
            targetHeading = state.getHeading();
        } else if (angleLock) {
            double heading = state.getHeading();
            double error = wrapRad(targetHeading - heading);

            double gyroRate = state.getChassisSpeedsRobot().omegaRadiansPerSecond;

            omegaCmd = KP * error - KD * gyroRate;
            omegaCmd = clamp(omegaCmd, -OMEGA_MAX, OMEGA_MAX);
        } else {
            omegaCmd = 0;
        }

        applyRobotVel(vx, vy, omegaCmd);
    }

    private void fieldCentric() {
        double vxF = deadband(map.forward, STICK_DEAD_BAND);
        double vyF = deadband(map.strafe, STICK_DEAD_BAND);
        double h = state.getHeading();
        double cos = Math.cos(h), sin = Math.sin(h);

        double vxR = vxF * cos + vyF * sin;
        double vyR = -vxF * sin + vyF * cos;

        double rotateStick = map.rotate;

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

    private void resetFieldCentric() {
        state.reset();
        targetHeading = 0.0;  // Reset angle lock target too
    }

    private static double deadband(double v, double d) { return Math.abs(v) > d ? v : 0.0; }
    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
    private static double wrapRad(double a) {
        while (a > Math.PI) a -= 2*Math.PI;
        while (a < -Math.PI) a += 2*Math.PI;
        return a;
    }
}
