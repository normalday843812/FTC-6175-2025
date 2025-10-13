package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

@TeleOp(name = "Hood Test", group = "Testing")
public class HoodTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hw = new RobotHardware(this);

        GamepadMap map = new GamepadMap(this);

        hw.initHood();
        Hood hood = new Hood(
                hw.getHoodServo(),
                map,
                this
        );

        if (isStopRequested()) return;

        waitForStart();

        while (opModeIsActive()) {
            map.update();
            hood.operate();
            TelemetryHelper.update();
        }
    }
}
