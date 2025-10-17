package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Simple Shooter Test", group = "Testing")
public class SimpleShooterTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");

        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        if (isStopRequested()) return;

        waitForStart();

        while (opModeIsActive()) {
            shooterMotor.setPower(gamepad1.right_trigger);
        }
    }
}
