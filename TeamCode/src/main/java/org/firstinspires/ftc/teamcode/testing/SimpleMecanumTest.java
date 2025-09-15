package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp
public class SimpleMecanumTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotHardware hw = new RobotHardware(this);
        hw.initHardware();


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            hw.getFrontLeft().setPower(frontLeftPower);
            hw.getBackLeft().setPower(backLeftPower);
            hw.getFrontRight().setPower(frontRightPower);
            hw.getBackRight().setPower(backRightPower);
        }
    }
}
