package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

public class RobotHardware {
    private final LinearOpMode inputOpMode;
//    // Constructor for OpMode
    public RobotHardware(LinearOpMode opMode) { inputOpMode = opMode; }
//
//    // Pinpoint
//    GoBildaPinpointDriver pinpoint;
//    public static double pinpointXOffset = -84.0; // TODO: replace with our actual values
//    public static double pinpointYOffset = -168.0; // TODO: replace with our actual values
//
//    // Webcam
//    private WebcamName webcam1;
//    private VisionPortal visionPortal;
//    public static final Size CAMERA_RES = new Size(640, 480);
//    public static final VisionPortal.StreamFormat STREAM_FORMAT = VisionPortal.StreamFormat.MJPEG;
//    public static final boolean ENABLE_LIVE_VIEW = false;
//
//    // Motors
//    // Drive
//    private DcMotorEx frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;
//    // Intake
//    private DcMotor intakeMotor;
    // Shooter
    private DcMotorEx shooterMotor;

    // Servos
    private Servo hoodServo;

    public void initHardware() {
        // Motors
        // Drive
//        frontRightMotor = inputOpMode.hardwareMap.get(DcMotorEx.class, "front_right_drive");
//        frontLeftMotor = inputOpMode.hardwareMap.get(DcMotorEx.class, "front_left_drive");
//        backRightMotor = inputOpMode.hardwareMap.get(DcMotorEx.class, "back_right_drive");
//        backLeftMotor = inputOpMode.hardwareMap.get(DcMotorEx.class, "back_left_drive");
//        // Intake
//        intakeMotor = inputOpMode.hardwareMap.get(DcMotor.class, "intake");
//        // Shooter
        shooterMotor = inputOpMode.hardwareMap.get(DcMotorEx.class, "shooter");
//
//        // Set directions
//        // Drive
//        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        // Set motor zero power behaviours
//        // Drive
//        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        // Intake
//        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Shooter
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set motor encoders
        // Drive
//        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        // Intake
//        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Shooter
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Servos
        hoodServo = inputOpMode.hardwareMap.get(Servo.class, "hood_servo");

//        // Pinpoint
//        pinpoint = inputOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//        configurePinpoint();
//
//        // Webcam
//        webcam1 = inputOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");
    }

    // Helpers
//    private void configurePinpoint(){
//        pinpoint.setOffsets(pinpointXOffset, pinpointYOffset, DistanceUnit.MM);
//        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//
//        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
//                GoBildaPinpointDriver.EncoderDirection.FORWARD); // TODO: Replace with our robot's configuration
//        pinpoint.resetPosAndIMU();
//    }

    // Webcam
//    public void initVision(VisionProcessor... processors) {
//        if (visionPortal != null) return;
//
//        VisionPortal.Builder builder = new VisionPortal.Builder()
//                .setCamera(webcam1)
//                .setCameraResolution(CAMERA_RES)
//                .setStreamFormat(STREAM_FORMAT)
//                .enableLiveView(ENABLE_LIVE_VIEW);
//
//        for (VisionProcessor p : processors) {
//            builder.addProcessor(p);
//        }
//
//        visionPortal = builder.build();
//    }
//    public void stopVisionStreaming() {
//        if (visionPortal != null) visionPortal.stopStreaming();
//    }
//    public void resumeVisionStreaming() {
//        if (visionPortal != null) visionPortal.resumeStreaming();
//    }
//    public void closeVision() {
//        if (visionPortal != null) {
//            visionPortal.close();
//            visionPortal = null;
//        }
//    }
//
//    // Getters
//    // Motors
//    public DcMotorEx getFrontRight() { return frontRightMotor; }
//    public DcMotorEx getFrontLeft() { return frontLeftMotor; }
//    public DcMotorEx getBackRight() { return backRightMotor; }
//    public DcMotorEx getBackLeft() { return backLeftMotor; }
//    public DcMotor getIntakeMotor() { return intakeMotor; }
    public DcMotorEx getShooterMotor() { return shooterMotor; }

    // Servos
    public Servo getHoodServo() { return hoodServo; }

//    // Pinpoint
//    public GoBildaPinpointDriver getPinpoint() { return pinpoint; }
//
//    // Webcam
//    public VisionPortal getVisionPortal() { return visionPortal; }
//    public WebcamName getWebcam1() { return webcam1; }

}
