package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.config.DriveConfig.BACK_LEFT_DIR;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.BACK_LEFT_NAME;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.BACK_RIGHT_DIR;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.BACK_RIGHT_NAME;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.FRONT_LEFT_DIR;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.FRONT_LEFT_NAME;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.FRONT_RIGHT_DIR;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.FRONT_RIGHT_NAME;
import static org.firstinspires.ftc.teamcode.config.GlobalConfig.isFailFastOnMissingHardware;
import static org.firstinspires.ftc.teamcode.config.LocalisationConfig.ENCODER_RESOLUTION;
import static org.firstinspires.ftc.teamcode.config.LocalisationConfig.FORWARD_ENCODER_DIRECTION;
import static org.firstinspires.ftc.teamcode.config.LocalisationConfig.PINPOINT_X_OFFSET_IN;
import static org.firstinspires.ftc.teamcode.config.LocalisationConfig.PINPOINT_Y_OFFSET_IN;
import static org.firstinspires.ftc.teamcode.config.LocalisationConfig.STRAFE_ENCODER_DIRECTION;
import static org.firstinspires.ftc.teamcode.config.RgbIndicatorConfig.GREEN_POS;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.shooterMotorDirection;
import static org.firstinspires.ftc.teamcode.config.TransferConfig.SERVO_1_DIRECTION;
import static org.firstinspires.ftc.teamcode.config.TransferConfig.SERVO_2_DIRECTION;

import android.util.Size;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.ShooterConfig;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

public class RobotHardware {
    private final OpMode inputOpMode;

    // Constructor for OpMode
    public RobotHardware(OpMode opMode) {
        inputOpMode = opMode;
    }

    public OpMode getOpMode() {
        return inputOpMode;
    }

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

    // Color sensor
    private NormalizedColorSensor intakeColorSensor;

    private Servo rgbIndicator;

    // IMU
    IMU imu;
    IMU.Parameters imuParams;

    // Motors
    // Drive
    private DcMotorEx frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;

    // Intake
    private DcMotorEx intakeMotor;

    // Shooter
    private DcMotorEx shooterMotor;

    // Shooter Yaw
    private DcMotorEx shooterYawMotor;

    // Servos
    private Servo hoodServo;
    private Servo spindexerServo;
    private Servo transferServo1, transferServo2;

    public void initPinpoint() {
        pinpoint = inputOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(PINPOINT_X_OFFSET_IN, PINPOINT_Y_OFFSET_IN, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(ENCODER_RESOLUTION);

        pinpoint.setEncoderDirections(FORWARD_ENCODER_DIRECTION,
                STRAFE_ENCODER_DIRECTION);
        pinpoint.resetPosAndIMU();
    }

    public void initIntakeColorSensor() {
        intakeColorSensor = inputOpMode.hardwareMap.get(NormalizedColorSensor.class, "intake_color_sensor");
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

    public void initTransfer() {
        transferServo1 = inputOpMode.hardwareMap.get(Servo.class, "transfer_servo_1");
        transferServo2 = inputOpMode.hardwareMap.get(Servo.class, "transfer_servo_2");
        transferServo1.setDirection(SERVO_1_DIRECTION);
        transferServo2.setDirection(SERVO_2_DIRECTION);
    }

    public void initWebcam() {
        webcam1 = inputOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");

    }

    public void initRgbIndicator() {
        rgbIndicator = inputOpMode.hardwareMap.get(Servo.class, "rgb_indicator");
        rgbIndicator.setPosition(GREEN_POS);
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
        if (limelight == null) {
            if (isFailFastOnMissingHardware()) {
                throw new IllegalStateException("Limelight is not initialized; cannot switch to pipeline " + pipelineNum);
            } else {
                initLimeLight(100);
                limelight.pipelineSwitch(pipelineNum);
            }
        } else {
            limelight.pipelineSwitch(pipelineNum);
        }
    }
    
    public void initDriveMotors() {
        frontRightMotor = inputOpMode.hardwareMap.get(DcMotorEx.class, FRONT_RIGHT_NAME);
        frontLeftMotor = inputOpMode.hardwareMap.get(DcMotorEx.class, FRONT_LEFT_NAME);
        backRightMotor = inputOpMode.hardwareMap.get(DcMotorEx.class, BACK_RIGHT_NAME);
        backLeftMotor = inputOpMode.hardwareMap.get(DcMotorEx.class, BACK_LEFT_NAME);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(FRONT_LEFT_DIR);
        frontRightMotor.setDirection(FRONT_RIGHT_DIR);
        backRightMotor.setDirection(BACK_RIGHT_DIR);
        backLeftMotor.setDirection(BACK_LEFT_DIR);
    }

    public void initIntake() {
        intakeMotor = inputOpMode.hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void initShooter() {
        shooterMotor = inputOpMode.hardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(shooterMotorDirection);
        shooterMotor.setVelocityPIDFCoefficients(
                ShooterConfig.P, ShooterConfig.I, ShooterConfig.D, ShooterConfig.F);
    }

    public void initShooterYaw() {
        shooterYawMotor = inputOpMode.hardwareMap.get(DcMotorEx.class, "shooter_yaw");
        shooterYawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterYawMotor.setTargetPosition(0);
        shooterYawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void initHood() {
        hoodServo = inputOpMode.hardwareMap.get(Servo.class, "hood_servo");
    }

    public void initSpindexer() {
        spindexerServo = inputOpMode.hardwareMap.get(Servo.class, "spindexer_servo");
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
        if (frontRightMotor == null) {
            if (isFailFastOnMissingHardware()) {
                throw new IllegalStateException("frontRightMotor not init");
            } else {
                initDriveMotors();
                return frontRightMotor;
            }
        } else {
            return frontRightMotor;
        }
    }

    public DcMotorEx getFrontLeft() {
        if (frontLeftMotor == null) {
            if (isFailFastOnMissingHardware()) {
                throw new IllegalStateException("frontLeftMotor not init");
            } else {
                initDriveMotors();
                return frontLeftMotor;
            }
        } else {
            return frontLeftMotor;
        }
    }

    public DcMotorEx getBackRight() {
        if (backRightMotor == null) {
            if (isFailFastOnMissingHardware()) {
                throw new IllegalStateException("backRightMotor not init");
            } else {
                initDriveMotors();
                return backRightMotor;
            }
        } else {
            return backRightMotor;
        }
    }

    public DcMotorEx getBackLeft() {
        if (backLeftMotor == null) {
            if (isFailFastOnMissingHardware()) {
                throw new IllegalStateException("backLeftMotor not init");
            } else {
                initDriveMotors();
                return backLeftMotor;
            }
        } else {
            return backLeftMotor;
        }
    }

    public DcMotorEx getIntakeMotor() {
        if (intakeMotor == null) {
            if (isFailFastOnMissingHardware()) {
                throw new IllegalStateException("intakeMotor not init");
            } else {
                initIntake();
                return intakeMotor;
            }
        } else {
            return intakeMotor;
        }
    }

    public DcMotorEx getShooterMotor() {
        if (shooterMotor == null) {
            if (isFailFastOnMissingHardware()) {
                throw new IllegalStateException("shooterMotor not init");
            } else {
                initShooter();
                return shooterMotor;
            }
        } else {
            return shooterMotor;
        }
    }

    public DcMotorEx getShooterYawMotor() {
        if (shooterYawMotor == null) {
            if (isFailFastOnMissingHardware()) {
                throw new IllegalStateException("shooterYawMotor not init");
            } else {
                initShooterYaw();
                return shooterYawMotor;
            }
        } else {
            return shooterYawMotor;
        }
    }

    // Odo encoders
    public DcMotorEx getOdoParallel() {
        if (frontRightMotor == null) {
            if (isFailFastOnMissingHardware()) {
                throw new IllegalStateException("frontRightMotor (OdoParallel) not init");
            } else {
                initDriveMotors();
                return frontRightMotor;
            }
        } else {
            return frontRightMotor;
        }
    }

    public DcMotorEx getOdoPerp() {
        if (backLeftMotor == null) {
            if (isFailFastOnMissingHardware()) {
                throw new IllegalStateException("backLeftMotor (odoPerp) not init");
            } else {
                initDriveMotors();
                return backLeftMotor;
            }
        } else {
            return backLeftMotor;
        }
    }

    // Misc
    public Servo getRgbIndicator() {
        if (rgbIndicator == null) {
            if (isFailFastOnMissingHardware()) {
                throw new IllegalStateException("rgbIndicator not init");
            } else {
                initRgbIndicator();
                return rgbIndicator;
            }
        } else {
            return rgbIndicator;
        }
    }

    // Servos
    public Servo getHoodServo() {
        if (hoodServo == null) {
            if (isFailFastOnMissingHardware()) {
                throw new IllegalStateException("hoodServo not init");
            } else {
                initHood();
                return hoodServo;
            }
        } else {
            return hoodServo;
        }
    }

    public Servo getTransferServo1() {
        if (transferServo1 == null) {
            if (isFailFastOnMissingHardware()) {
                throw new IllegalStateException("transferServo not init");
            } else {
                initTransfer();
                return transferServo1;
            }
        } else {
            return transferServo1;
        }
    }

    public Servo getTransferServo2() {
        if (transferServo2 == null) {
            if (isFailFastOnMissingHardware()) {
                throw new IllegalStateException("transferServo not init");
            } else {
                initTransfer();
                return transferServo2;
            }
        } else {
            return transferServo2;
        }
    }

    public Servo getSpindexerServo() {
        if (spindexerServo == null) {
            if (isFailFastOnMissingHardware()) {
                throw new IllegalStateException("spindexerServo not init");
            } else {
                initSpindexer();
                return spindexerServo;
            }
        } else {
            return spindexerServo;
        }
    }

    public NormalizedColorSensor getIntakeColorSensor() {
        if (intakeColorSensor == null) {
            if (isFailFastOnMissingHardware()) {
                throw new IllegalStateException("intakeColorSensor not init");
            } else {
                initIntakeColorSensor();
                return intakeColorSensor;
            }
        } else {
            return intakeColorSensor;
        }
    }

    // Pinpoint
    public GoBildaPinpointDriver getPinpoint() {
        if (pinpoint == null) {
            if (isFailFastOnMissingHardware()) {
                throw new IllegalStateException("pinpoint not init");
            } else {
                initPinpoint();
                return pinpoint;
            }
        } else {
            return pinpoint;
        }
    }

    // IMU
    public IMU getIMU() {
        if (imu == null) {
            if (isFailFastOnMissingHardware()) {
                throw new IllegalStateException("imu not init");
            } else {
                initIMU();
                return imu;
            }
        } else {
            return imu;
        }
    }

    // Webcam
    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    public WebcamName getWebcam1() {
        return webcam1;
    }

    // Limelight
    public Limelight3A getLimelight() {
        if (limelight == null) {
            if (isFailFastOnMissingHardware()) {
                throw new IllegalStateException("limelight not init");
            } else {
                initLimeLight(100);
                return limelight;
            }
        } else {
            return limelight;
        }
    }
}
