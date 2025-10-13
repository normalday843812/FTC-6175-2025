package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.config.DriveConfig.*;

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
    public static final FollowerConstants followerConstants =
            new FollowerConstants()
                    .mass(5.0); // TODO: put actual robot mass (kg)

    public static final MecanumConstants driveConstants =
            new MecanumConstants()
                    .maxPower(1.0)
                    .rightFrontMotorName("front_right_drive")
                    .rightRearMotorName("back_right_drive")
                    .leftRearMotorName("back_left_drive")
                    .leftFrontMotorName("front_left_drive")
                    .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static final PinpointConstants localizerConstants =
            new PinpointConstants()
                    .forwardPodY(PINPOINT_Y_OFFSET_M)
                    .strafePodX(PINPOINT_X_OFFSET_M)
                    .distanceUnit(DistanceUnit.METER)
                    .hardwareMapName("pinpoint")
                    .encoderResolution(ENCODER_RESOLUTION)
                    .forwardEncoderDirection(FORWARD_ENCODER_DIRECTION)
                    .strafeEncoderDirection(STRAFE_ENCODER_DIRECTION);

    public static final PathConstraints pathConstraints =
            new PathConstraints(
                    0.80,
                    60.0,
                    1.00,
                    1.00
            );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
