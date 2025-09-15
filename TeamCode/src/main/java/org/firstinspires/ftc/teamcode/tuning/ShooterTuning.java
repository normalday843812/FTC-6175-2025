package org.firstinspires.ftc.teamcode.tuning;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.localisation.StateEstimator;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalizer;

@Configurable
@TeleOp(name="Shooter Tuning", group="Tuning")
public class ShooterTuning extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hw = new RobotHardware(this);
        hw.initHardware();

        AprilTagLocalizer aprilTagLocalizer = new AprilTagLocalizer(
                this
        );

        aprilTagLocalizer.initAprilTag();

        StateEstimator state = new StateEstimator(
                this,
                hw.getPinpoint(),
                aprilTagLocalizer
        );

        hw.initVision(aprilTagLocalizer.getAprilTag());

        if (isStopRequested()) return;

        waitForStart();

        while (opModeIsActive()) {
            state.update();
            telemetry.update();
        }
    }
}
