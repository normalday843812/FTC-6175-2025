package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.localisation.StateEstimator;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.vision.AprilTagLocalizer;

@Autonomous(name = "Auto test", group = "Pedro")
public class AutoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware
        RobotHardware hw = new RobotHardware(this);

        // Vision
        AprilTagLocalizer ll = new AprilTagLocalizer(hw.getLimelight());
        hw.initLimeLight(100);
        hw.setLimelightPipeline(0);

        // Localisation
        hw.initPinpoint();
        StateEstimator state = new StateEstimator(this, hw.getPinpoint(), ll);

        Follower follower = Constants.createFollower(hw, state);

        Pose startPose = new Pose(0, 0, 0);
        follower.setStartingPose(startPose);

        PathChain chain = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(0, 0),
                        new Pose(36, 0)
                ))
                .setConstantHeadingInterpolation(0)
                .build();

        if (isStopRequested()) return;
        waitForStart();

        follower.followPath(chain, true);

        // Main loop
        while (opModeIsActive()) {
            follower.update();

            // End-of-path reached and no longer busy
            if (!follower.isBusy()) {
                // Do whatever here
                break;
            }
        }
    }
}