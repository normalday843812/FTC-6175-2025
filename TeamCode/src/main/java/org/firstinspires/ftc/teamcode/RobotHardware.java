package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotHardware {
    private final LinearOpMode inputOpMode;
    // Constructor for OpMode
    public RobotHardware(LinearOpMode opMode) { inputOpMode = opMode; }

    // Pinpoint
    GoBildaPinpointDriver pinpoint;
    public static double pinpointXOffset = -84.0; // TODO: replace with our actual values
    public static double pinpointYOffset = -168.0; // TODO: replace with our actual values

    // Motors
    private DcMotorEx frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;
    public void initHardware() {
        // Motors
        frontRightMotor = inputOpMode.hardwareMap.get(DcMotorEx.class, "front_right_drive");
        frontLeftMotor = inputOpMode.hardwareMap.get(DcMotorEx.class, "front_left_drive");
        backRightMotor = inputOpMode.hardwareMap.get(DcMotorEx.class, "back_right_drive");
        backLeftMotor = inputOpMode.hardwareMap.get(DcMotorEx.class, "back_left_drive");
        
        // Set directions
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motor zero power behaviours
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motor encoders
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Pinpoint
        pinpoint = inputOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();
    }

    public void configurePinpoint(){
        pinpoint.setOffsets(pinpointXOffset, pinpointYOffset, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD); // TODO: Replace with our robot's configuration
        pinpoint.resetPosAndIMU();
    }


    // Getters
    public DcMotorEx getFrontRight() { return frontRightMotor; }
    public DcMotorEx getFrontLeft()  { return frontLeftMotor; }
    public DcMotorEx getBackRight()  { return backRightMotor; }
    public DcMotorEx getBackLeft()   { return backLeftMotor; }

    public GoBildaPinpointDriver getPinpoint() { return pinpoint; }
}
