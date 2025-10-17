package org.firstinspires.ftc.teamcode.pedroPathing;

import androidx.annotation.NonNull;

import com.pedropathing.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.localisation.StateEstimator;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;

import java.util.function.DoubleSupplier;

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
        hw.initDriveMotors();
        Drivetrain drivetrain = getDrivetrain(hw);

        // TODO: After running the velocity tuners, set measured max speeds in in/s:
        // follower.setXVelocity(<forward_ips>);
        // follower.setYVelocity(<strafe_ips>);

        Follower follower = new Follower(followerConstants, state, drivetrain, pathConstraints);
        follower.getDrivetrain().setNominalVoltage(12.0);
        follower.getDrivetrain().useVoltageCompensation(true);
        return follower;
    }

    @NonNull
    private static Drivetrain getDrivetrain(RobotHardware hw) {
        DcMotorEx fl = hw.getFrontLeft();
        DcMotorEx fr = hw.getFrontRight();
        DcMotorEx bl = hw.getBackLeft();
        DcMotorEx br = hw.getBackRight();

        DoubleSupplier vsup = () -> {
            double best = 0.0;
            for (VoltageSensor vs : hw.getOpMode().hardwareMap.voltageSensor) {
                double v = vs.getVoltage();
                if (!Double.isNaN(v) && v > best) best = v;
            }
            return best;
        };

        return new Mecanum.PedroMecanumDrivetrain(fl, fr, bl, br, vsup);
    }
}