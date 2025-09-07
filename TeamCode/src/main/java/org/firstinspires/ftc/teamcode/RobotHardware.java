package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class RobotHardware {
    private LinearOpMode inputOpMode = null;
    // Constructor for OpMode
    public RobotHardware(LinearOpMode opMode) { inputOpMode = opMode; }

    private DcMotorEx frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor;
    public void Init() {
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
    }

    public DcMotorEx getFrontRight() { return frontRightMotor; }
    public DcMotorEx getFrontLeft()  { return frontLeftMotor; }
    public DcMotorEx getBackRight()  { return backRightMotor; }
    public DcMotorEx getBackLeft()   { return backLeftMotor; }
}
