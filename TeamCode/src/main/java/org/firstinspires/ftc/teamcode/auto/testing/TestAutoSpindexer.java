package org.firstinspires.ftc.teamcode.auto.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

@Autonomous(name = "Spindexer auto test", group = "Testing")
public class TestAutoSpindexer extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware hw = new RobotHardware(this);
        hw.initSpindexer();
        Spindexer sp = new Spindexer(hw.getSpindexerServo(), null, this);
        sp.startAuto();

        if (isStopRequested()) return;
        waitForStart();

        // step a few times
        for (int i = 0; i < 3 && opModeIsActive(); i++) {
            sp.stepForward();
            sp.operate();
            TelemetryHelper.update();
            sleep(700);
        }
        while (opModeIsActive()) {
            sp.operate();
            TelemetryHelper.update();
            sleep(50);
        }
    }
}