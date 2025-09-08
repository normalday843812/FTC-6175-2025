package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.localisation.StateEstimator;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;

@TeleOp
public class InitialTeleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hw = new RobotHardware(this);
        hw.initHardware();

        StateEstimator state = new StateEstimator(hw.getPinpoint());

        Mecanum drive = new Mecanum(
                hw.getFrontLeft(),
                hw.getBackLeft(),
                hw.getFrontRight(),
                hw.getBackRight(),
                state,
                this
        );

        waitForStart();

        while (opModeIsActive()) {
            state.update();
            drive.operateMecanum();
            telemetry.update();
        }
    }
}
