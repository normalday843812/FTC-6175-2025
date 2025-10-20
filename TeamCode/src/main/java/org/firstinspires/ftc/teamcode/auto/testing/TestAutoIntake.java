package org.firstinspires.ftc.teamcode.auto.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

@Autonomous(name = "Auto Intake Test", group = "Testing")
public class TestAutoIntake extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware hw = new RobotHardware(this);
        hw.initIntake();
        Intake intake = new Intake(hw.getIntakeMotor(), null, this);
        intake.startAuto();
        intake.setAutoMode(Intake.AutoMode.OFF);

        if (isStopRequested()) return;
        waitForStart();

        long t0 = System.currentTimeMillis();
        while (opModeIsActive()) {
            double t = (System.currentTimeMillis() - t0) / 1000.0;
            if (t < 2.0) intake.setAutoMode(Intake.AutoMode.FORWARD);
            else intake.setAutoMode(Intake.AutoMode.OFF);

            intake.operate();
            TelemetryHelper.update();
            sleep(50);
        }
    }
}