package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="RGB Indicator Test", group="Testing")
public class RgbIndicatorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo rgb = hardwareMap.get(Servo.class, "rgb_indicator");
//        if (rgb instanceof PwmControl) {
//            ((PwmControl) rgb).setPwmRange(new PwmControl.PwmRange(500, 2500));
//        }
        rgb.setPosition(0.500);

        waitForStart();

        while (opModeIsActive()) {
            rgb.setPosition(gamepad1.right_trigger);
        }
    }
}
