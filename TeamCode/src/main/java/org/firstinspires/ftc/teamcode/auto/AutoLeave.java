package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="Leave w/ pedro", group="A")
public class AutoLeave extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.createFollower(hardwareMap);

        Pose start  = Constants.START_POSE;
        Pose leave  = new Pose(start.getX(), start.getY() + 24.0, start.getHeading());
        Pose settle = new Pose(leave.getX() + 6.0, leave.getY(),  leave.getHeading());

        Pose prev = new Pose(start.getX(), start.getY() - 6.0, start.getHeading());

        PathChain chain = new PathBuilder(follower, Constants.pathConstraints)
                .setGlobalConstantHeadingInterpolation(start.getHeading())
                .curveThrough(prev, start, /*tension=*/0.0, leave, settle)
                .build();

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        follower.followPath(chain);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            Pose p = follower.getPose();
            telemetry.addData("Pose", "(%.1f, %.1f, %.1f°)", p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
            telemetry.update();
        }

        follower.breakFollowing();  // stop motors
        while (opModeIsActive()) idle();
    }
}
