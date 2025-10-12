package org.firstinspires.ftc.teamcode.subsystems;

import static androidx.core.math.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.*;
import static org.firstinspires.ftc.teamcode.config.HoodServoConfig.TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.util.MathUtil.deadband;
import static org.firstinspires.ftc.teamcode.util.MathUtil.wrapRad;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.localisation.StateEstimator;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

import java.util.stream.Stream;

@Configurable
public class Mecanum {
    private final OpMode opmode;

    // Motors
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // State estimation
    private final StateEstimator state;

    // Controls
    private final GamepadMap map;

    // Telemetry
    private final TelemetryHelper tele;

    // Angle and slow mode modifiers
    private boolean angleLock = false, slowMode = false, fieldCentricEnabled = false;
    private double targetHeading = 0;

    // Constructor
    public Mecanum(DriveMotors driveMotors, StateEstimator state, GamepadMap map, OpMode opmode) {
        this.frontLeft = driveMotors.frontLeft;
        this.frontRight = driveMotors.frontRight;
        this.backLeft = driveMotors.backLeft;
        this.backRight = driveMotors.backRight;
        this.state = state;
        this.map = map;
        this.opmode = opmode;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
    }

    public void operate() {
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
        tele.addLine("--- Mecanum ---")
                .addData("FL Power:", "%.2f", frontLeft.getPower())
                .addData("BL Power:", "%.2f", backLeft.getPower())
                .addData("FR Power:", "%.2f", frontRight.getPower())
                .addData("BR Power:", "%.2f", backRight.getPower())
                .addData("Control:", "%s", fieldCentricEnabled ? "Field" : "Robot")
                .addData("Slow Mode:", "%b", slowMode)
                .addData("Angle Lock:", "%b", angleLock)
                .addData("Target Heading:", "%.1f°", Math.toDegrees(targetHeading));
    }

    private void handleToggles() {
        if (map.angleLockToggle) {
            angleLock = !angleLock;
            if (angleLock) { targetHeading = state.getFusedHeading(AngleUnit.RADIANS); }
        }
        if (map.slowModeToggle) { slowMode = !slowMode; }
        if (map.fieldCentricToggle) { fieldCentricEnabled = !fieldCentricEnabled; }
        if (map.stateEstimatorFallbackToggle) { state.toggleFallbackMode(); }
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

        double maxMag = Stream.of(Math.abs(frontLeftPower), Math.abs(backLeftPower),
                        Math.abs(frontRightPower), Math.abs(backRightPower))
                .max(Double::compare).orElse(0.0);
        double scale = Math.max(1.0, maxMag);
        frontLeftPower /= scale; backLeftPower /= scale;
        frontRightPower /= scale; backRightPower /= scale;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    private void robotCentric() {
        double vx = deadband(map.forward, STICK_DB);
        double vy = deadband(map.strafe, STICK_DB);
        double rotateStick = map.rotate;

        double omegaCmd;
        boolean driverRotating = Math.abs(rotateStick) > ROT_DB;

        if (driverRotating) {
            omegaCmd = rotateStick;
            targetHeading = state.getFusedHeading(AngleUnit.RADIANS);
        } else if (angleLock) {
            double heading = state.getFusedHeading(AngleUnit.RADIANS);
            double error = wrapRad(targetHeading - heading);

            double gyroRate = state.getChassisSpeedsRobot().omegaRadiansPerSecond;

            omegaCmd = KP_YAW * error - KD_YAW * gyroRate;
            omegaCmd = clamp(omegaCmd, -OMEGA_MAX, OMEGA_MAX);
        } else {
            omegaCmd = 0;
        }

        applyRobotVel(vx, vy, omegaCmd);
    }

    private void fieldCentric() {
        double vxF = deadband(map.forward, STICK_DB);
        double vyF = deadband(map.strafe, STICK_DB);
        double h = state.getFusedHeading(AngleUnit.RADIANS);
        double cos = Math.cos(h), sin = Math.sin(h);

        double vxR = vxF * cos + vyF * sin;
        double vyR = -vxF * sin + vyF * cos;

        double rotateStick = map.rotate;

        double omegaCmd;
        if (Math.abs(rotateStick) > ROT_DB) {
            omegaCmd = rotateStick;
            targetHeading = state.getFusedHeading(AngleUnit.RADIANS);
        } else if (angleLock) {
            double gyroRate = state.getChassisSpeedsRobot().omegaRadiansPerSecond;
            double err = wrapRad(targetHeading - state.getFusedHeading(AngleUnit.RADIANS));
            omegaCmd = clamp(KP_YAW * err - KD_YAW * gyroRate, -OMEGA_MAX, OMEGA_MAX);
        } else {
            omegaCmd = 0.0;
        }

        applyRobotVel(vxR, vyR, omegaCmd);
    }

    private void resetFieldCentric() {
        state.reset();
        targetHeading = 0.0;  // Reset angle lock target too
    }
}
