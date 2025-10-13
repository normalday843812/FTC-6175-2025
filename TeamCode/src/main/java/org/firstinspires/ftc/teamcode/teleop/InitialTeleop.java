package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.localisation.StateEstimator;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalizer;

@TeleOp
public class InitialTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hw = new RobotHardware(this);

        // Vision
        AprilTagLocalizer aprilTagLocalizer = new AprilTagLocalizer(hw.getLimelight());
        hw.initLimeLight(100);
        hw.setLimelightPipeline(0);

        // Odometry / IMU
        hw.initPinpoint();
        StateEstimator state = new StateEstimator(this, hw.getPinpoint(), aprilTagLocalizer);

        Follower follower = Constants.createFollower(hw, state);

        GamepadMap map = new GamepadMap(this);

        // Drive
        hw.initDriveMotors();
        Mecanum drive = new Mecanum(state, map, this, follower);

        // Subsystems
        hw.initIntake();
        Intake intake = new Intake(hw.getIntakeMotor(), map, this);

        hw.initShooter();
        Shooter shooter = new Shooter(hw.getShooterMotor(), map, this);

        hw.initHood();
        Hood hood = new Hood(hw.getHoodServo(), map, this);

        if (isStopRequested()) return;
        waitForStart();

        while (opModeIsActive()) {
            map.update();
            state.update();
            drive.operate();
            intake.operate();
            shooter.operate();
            hood.operate();
            telemetry.update();
        }
    }
}
