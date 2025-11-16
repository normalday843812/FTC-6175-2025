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
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.SHOOTER_MOTOR_DIRECTION;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.SHOOTER_MOTOR_DIRECTION_1;
import static org.firstinspires.ftc.teamcode.config.TransferConfig.SERVO_1_DIRECTION;
import static org.firstinspires.ftc.teamcode.config.TransferConfig.SERVO_2_DIRECTION;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.ShooterConfig;

public class RobotHardware {
    private final OpMode inputOpMode;

    // Constructor for OpMode
    public RobotHardware(OpMode opMode) {
        inputOpMode = opMode;
    }

    // Pinpoint
    GoBildaPinpointDriver pinpoint;

    // Limelight
    private Limelight3A limelight;

    // Motors
    // Drive
    private DcMotorEx frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;

    // Intake
    private DcMotorEx intakeMotor;

    // Shooter
    private DcMotorEx shooterMotor, shooterMotor1;

    // Shooter Yaw
    private DcMotorEx shooterYawMotor;

    // Servos
    private Servo hoodServo;
    private Servo spindexerServo;
    private Servo transferServo1;
    private CRServo transferServo2;
    private Servo rgbIndicator;

    private NormalizedColorSensor slotColor0, slotColor1, slotColor2;

    private DistanceSensor frontDistance;

    public void initPinpoint() {
        pinpoint = inputOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(PINPOINT_X_OFFSET_IN, PINPOINT_Y_OFFSET_IN, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(ENCODER_RESOLUTION);

        pinpoint.setEncoderDirections(FORWARD_ENCODER_DIRECTION,
                STRAFE_ENCODER_DIRECTION);
        pinpoint.resetPosAndIMU();
    }

    public void initTransfer() {
        transferServo1 = inputOpMode.hardwareMap.get(Servo.class, "transfer_servo_1");
        transferServo2 = inputOpMode.hardwareMap.get(CRServo.class, "transfer_servo_2");
        transferServo1.setDirection(SERVO_1_DIRECTION);
        transferServo2.setDirection(SERVO_2_DIRECTION);
    }

    public void initRgbIndicator() {
        rgbIndicator = inputOpMode.hardwareMap.get(Servo.class, "rgb_indicator");
        rgbIndicator.setPosition(GREEN_POS);
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
        shooterMotor1 = inputOpMode.hardwareMap.get(DcMotorEx.class, "shooter1");
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(SHOOTER_MOTOR_DIRECTION);
        shooterMotor1.setDirection(SHOOTER_MOTOR_DIRECTION_1);
        shooterMotor.setVelocityPIDFCoefficients(
                ShooterConfig.P, ShooterConfig.I, ShooterConfig.D, ShooterConfig.F);
        shooterMotor1.setVelocityPIDFCoefficients(
                ShooterConfig.P, ShooterConfig.I, ShooterConfig.D, ShooterConfig.F);
    }

    public void initShooterYaw() {
        shooterYawMotor = inputOpMode.hardwareMap.get(DcMotorEx.class, "shooter_yaw");
        shooterYawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterYawMotor.setTargetPosition(0);
        shooterYawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void initHood() {
        hoodServo = inputOpMode.hardwareMap.get(Servo.class, "hood_servo");
    }

    public void initSpindexer() {
        spindexerServo = inputOpMode.hardwareMap.get(Servo.class, "spindexer_servo");
    }

    public void initSpindexColorSensors() {
        slotColor0 = inputOpMode.hardwareMap.get(NormalizedColorSensor.class, "spindex_color_0");
        slotColor1 = inputOpMode.hardwareMap.get(NormalizedColorSensor.class, "spindex_color_1");
        slotColor2 = inputOpMode.hardwareMap.get(NormalizedColorSensor.class, "spindex_color_2");
    }

    public void initFrontDistance() {
        frontDistance = inputOpMode.hardwareMap.get(DistanceSensor.class, "front_distance");
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

    public DcMotorEx getShooterMotor1() {
        if (shooterMotor1 == null) {
            if (isFailFastOnMissingHardware()) {
                throw new IllegalStateException("shooterMotor1 not init");
            } else {
                initShooter();
                return shooterMotor1;
            }
        } else {
            return shooterMotor1;
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

    public CRServo getTransferServo2() {
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

    public NormalizedColorSensor getSlotColor0() {
        return slotColor0;
    }

    public NormalizedColorSensor getSlotColor1() {
        return slotColor1;
    }

    public NormalizedColorSensor getSlotColor2() {
        return slotColor2;
    }

    public DistanceSensor getFrontDistance() {
        return frontDistance;
    }
}