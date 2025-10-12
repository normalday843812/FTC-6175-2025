package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.config.DriveConfig.FORWARD_ENCODER_DIRECTION;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.PINPOINT_X_OFFSET_M;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.PINPOINT_Y_OFFSET_M;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.STRAFE_ENCODER_DIRECTION;

import android.util.Size;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.ShooterConfig;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.Objects;

public class RobotHardware {
    private final LinearOpMode inputOpMode;
    // Constructor for OpMode
    public RobotHardware(LinearOpMode opMode) { inputOpMode = opMode; }

    // Pinpoint
    GoBildaPinpointDriver pinpoint;

    // Webcam
    private WebcamName webcam1;
    private VisionPortal visionPortal;
    public static final Size CAMERA_RES = new Size(640, 480);
    public static final VisionPortal.StreamFormat STREAM_FORMAT = VisionPortal.StreamFormat.MJPEG;
    public static final boolean ENABLE_LIVE_VIEW = false;

    // Limelight
    private Limelight3A limelight;

    // IMU
    IMU imu;
    IMU.Parameters imuParams;

    // Motors
    // Drive
    private DcMotorEx frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;
    // Intake
    private DcMotor intakeMotor;

    // Shooter
    private DcMotorEx shooterMotor;

    // Servos
    private Servo hoodServo;

    public void initPinpoint() {
        pinpoint = inputOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(PINPOINT_X_OFFSET_M, PINPOINT_Y_OFFSET_M, DistanceUnit.METER);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(FORWARD_ENCODER_DIRECTION,
                STRAFE_ENCODER_DIRECTION);
        pinpoint.resetPosAndIMU();
    }

    public void initIMU() {
        imu = inputOpMode.hardwareMap.get(IMU.class, "imu");

        imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(imuParams);
    }

    public void initWebcam() {
        webcam1 = inputOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");

    }

    public void initVision(CameraName webcam, VisionProcessor... processors) {
        if (visionPortal != null) return;

        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(CAMERA_RES)
                .setStreamFormat(STREAM_FORMAT)
                .enableLiveView(ENABLE_LIVE_VIEW);

        for (VisionProcessor p : processors) {
            builder.addProcessor(p);
        }

        visionPortal = builder.build();
    }

    public void initLimeLight(int pollRateHz) {
        limelight = inputOpMode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(pollRateHz);
        limelight.start();
    }

    public void setLimelightPipeline(int pipelineNum) {
        try {
            limelight.pipelineSwitch(pipelineNum);
        } catch (NullPointerException e) {
            throw new IllegalStateException("Limelight is not initialized; cannot switch to pipeline " + pipelineNum, e);
        }
    }

    public void initDriveMotors() {
        frontRightMotor = inputOpMode.hardwareMap.get(DcMotorEx.class, "front_right_drive");
        frontLeftMotor = inputOpMode.hardwareMap.get(DcMotorEx.class, "front_left_drive");
        backRightMotor = inputOpMode.hardwareMap.get(DcMotorEx.class, "back_right_drive");
        backLeftMotor = inputOpMode.hardwareMap.get(DcMotorEx.class, "back_left_drive");

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // If using two wheel: odom encoders given frontRightMotor is port 0 and backLeftMotor is port 3
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Odom pod
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Odom pod
    }

    public void initIntake() {
        intakeMotor = inputOpMode.hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void initShooter() {
        shooterMotor = inputOpMode.hardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setVelocityPIDFCoefficients(
                ShooterConfig.P, ShooterConfig.I, ShooterConfig.D, ShooterConfig.F);
    }

    public void initHood() {
        hoodServo = inputOpMode.hardwareMap.get(Servo.class, "hood_servo");
    }

    // Webcam
    public void stopVisionStreaming() {
        if (visionPortal != null) visionPortal.stopStreaming();
    }
    public void resumeVisionStreaming() {
        if (visionPortal != null) visionPortal.resumeStreaming();
    }
    public void closeVision() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
    }

    // Getters
    // Motors
    public DcMotorEx getFrontRight() {
        try {
            return Objects.requireNonNull(frontRightMotor, "frontRightMotor not init");
        } catch (NullPointerException e) {
            initDriveMotors();
            return frontRightMotor;
        }
    }
    public DcMotorEx getFrontLeft() {
        try {
            return Objects.requireNonNull(frontLeftMotor, "frontLeftMotor not init");
        } catch (NullPointerException e) {
            initDriveMotors();
            return frontLeftMotor;
        }
    }
    public DcMotorEx getBackRight() {
        try {
            return Objects.requireNonNull(backRightMotor, "backRightMotor not init");
        } catch (NullPointerException e) {
            initDriveMotors();
            return backRightMotor;
        }
    }
    public DcMotorEx getBackLeft() {
        try {
            return Objects.requireNonNull(backLeftMotor, "backLeftMotor not init");
        } catch (NullPointerException e) {
            initDriveMotors();
            return backLeftMotor;
        }
    }
    public DcMotor getIntakeMotor() {
        try {
            return Objects.requireNonNull(intakeMotor, "intakeMotor not init");
        } catch (NullPointerException e) {
            initIntake();
            return intakeMotor;
        }
    }

    public DcMotorEx getShooterMotor() {
        try {
            return Objects.requireNonNull(shooterMotor, "shooterMotor not init");
        } catch (NullPointerException e) {
            initShooter();
            return shooterMotor;
        }
    }

    // Odo encoders
    public DcMotorEx getOdoParallel() {
        try {
            return Objects.requireNonNull(frontRightMotor, "frontRightMotor not init");
        } catch (NullPointerException e) {
            initDriveMotors();
            return frontRightMotor;
        }
    }
    public DcMotorEx getOdoPerp() {
        try {
            return Objects.requireNonNull(backLeftMotor, "backLeftMotor not init");
        } catch (NullPointerException e) {
            initDriveMotors();
            return backLeftMotor;
        }
    }

    // Servos
    public Servo getHoodServo() {
        try {
            return Objects.requireNonNull(hoodServo, "hoodServo not init");
        } catch (NullPointerException e) {
            initHood();
            return hoodServo;
        }
    }

    // Pinpoint
    public GoBildaPinpointDriver getPinpoint() {
        try {
            return Objects.requireNonNull(pinpoint, "pinpoint not init");
        } catch (NullPointerException e) {
            initPinpoint();
            return pinpoint;
        }
    }

    // IMU
    public IMU getIMU() {
        try {
            return Objects.requireNonNull(imu, "imu not init");
        } catch (NullPointerException e) {
            initIMU();
            return imu;
        }
    }

    // Webcam
    public VisionPortal getVisionPortal() { return visionPortal; }
    public WebcamName getWebcam1() { return webcam1; }

    // Limelight
    public Limelight3A getLimelight() {
        try {
            return Objects.requireNonNull(limelight, "Limelight not init");
        } catch (NullPointerException e) {
            initLimeLight(100);
            return limelight;
        }
    }
}
