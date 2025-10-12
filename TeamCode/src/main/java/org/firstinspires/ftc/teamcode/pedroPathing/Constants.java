package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.config.DriveConfig.ENCODER_RESOLUTION;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.FORWARD_ENCODER_DIRECTION;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.STRAFE_ENCODER_DIRECTION;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants().mass(5.0); // TODO: put our robot's actual mass

    // Can't use RobotHardware because not an OpMode
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            .rightFrontMotorName("front_right_drive")
            .rightRearMotorName("back_right_drive")
            .leftRearMotorName("back_left_drive")
            .leftFrontMotorName("front_left_drive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD); // TODO: Input actual motor directions
            // TODO: Add actual tuning constants: https://pedropathing.com/docs/pathing/tuning/automatic

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-6) // TODO: tune
            .strafePodX(0.0) // TODO
            .distanceUnit(DistanceUnit.METER)
            .hardwareMapName("pinpoint")
            .encoderResolution(ENCODER_RESOLUTION)
            .forwardEncoderDirection(FORWARD_ENCODER_DIRECTION) // TODO
            .strafeEncoderDirection(STRAFE_ENCODER_DIRECTION); // TODO

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}