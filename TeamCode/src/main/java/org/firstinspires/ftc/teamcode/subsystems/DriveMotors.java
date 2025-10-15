package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DriveMotors {
    final DcMotorEx frontLeft;
    final DcMotorEx frontRight;
    final DcMotorEx backLeft;
    final DcMotorEx backRight;

    public DriveMotors(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }
}
