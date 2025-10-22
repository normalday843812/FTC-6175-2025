package org.firstinspires.ftc.teamcode.auto.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

@Autonomous(name = "Hood auto test", group = "Testing")
public class TestAutoHood extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware hw = new RobotHardware(this);
        hw.initHood();
        Hood hood = new Hood(hw.getHoodServo(), null, this);
        hood.startAuto();
        hood.setAutoTarget(0.5);

        if (isStopRequested()) return;
        waitForStart();

        while (opModeIsActive()) {
            hood.operate();
            TelemetryHelper.update();
            sleep(50);
        }
    }
}