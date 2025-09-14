package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.localisation.StateEstimator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;

@TeleOp
public class InitialTeleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hw = new RobotHardware(this);
        hw.initHardware();

        StateEstimator state = new StateEstimator(hw.getPinpoint());
        GamepadMap map = new GamepadMap(this);

        Mecanum drive = new Mecanum(
                hw.getFrontLeft(),
                hw.getBackLeft(),
                hw.getFrontRight(),
                hw.getBackRight(),
                state,
                map,
                this
        );

        Intake intake = new Intake(
                hw.getIntakeMotor(),
                map,
                this
        );

        if (isStopRequested()) return;

        waitForStart();

        while (opModeIsActive()) {
            map.update();
            state.update();
            drive.operateMecanum();
            intake.OperateIntake();
            telemetry.update();
        }
    }
}
