package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.localisation.StateEstimator;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5.0);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.007,
            100,
            1.0,
            10,
            1.0
    );

    public static Follower createFollower(RobotHardware hw, StateEstimator state) {
        // Ensure motors are initialized before we build the drivetrain
        hw.initDriveMotors();
        DcMotorEx fl = hw.getFrontLeft();
        DcMotorEx fr = hw.getFrontRight();
        DcMotorEx bl = hw.getBackLeft();
        DcMotorEx br = hw.getBackRight();

        Drivetrain drivetrain = new Mecanum.PedroMecanumDrivetrain(fl, fr, bl, br);

        // TODO: After running the velocity tuners, set measured max speeds in in/s:
        // follower.setXVelocity(<forward_ips>);
        // follower.setYVelocity(<strafe_ips>);

        return new Follower(followerConstants, state, drivetrain, pathConstraints);
    }
}