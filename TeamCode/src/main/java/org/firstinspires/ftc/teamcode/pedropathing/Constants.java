package org.firstinspires.ftc.teamcode.pedropathing;

import static org.firstinspires.ftc.teamcode.config.DriveConfig.BACK_LEFT_DIR;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.BACK_LEFT_NAME;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.BACK_RIGHT_DIR;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.BACK_RIGHT_NAME;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.FRONT_LEFT_DIR;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.FRONT_LEFT_NAME;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.FRONT_RIGHT_DIR;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.FRONT_RIGHT_NAME;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.ROBOT_MASS;
import static org.firstinspires.ftc.teamcode.config.LocalisationConfig.ENCODER_RESOLUTION;
import static org.firstinspires.ftc.teamcode.config.LocalisationConfig.FORWARD_ENCODER_DIRECTION;
import static org.firstinspires.ftc.teamcode.config.LocalisationConfig.PINPOINT_X_OFFSET_IN;
import static org.firstinspires.ftc.teamcode.config.LocalisationConfig.PINPOINT_Y_OFFSET_IN;
import static org.firstinspires.ftc.teamcode.config.LocalisationConfig.STRAFE_ENCODER_DIRECTION;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(ROBOT_MASS)
            .forwardZeroPowerAcceleration(-31.521)
            .lateralZeroPowerAcceleration(-47.84)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.05, 0.028))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(3, 0, 0.06, 0.02))
            .useSecondaryTranslationalPIDF(true)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0.015, 0.015))
            .useSecondaryHeadingPIDF(true);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .xVelocity(70.46433)
            .yVelocity(31.76)
            .maxPower(1)
            .rightFrontMotorName(FRONT_RIGHT_NAME)
            .rightRearMotorName(BACK_RIGHT_NAME)
            .leftRearMotorName(BACK_LEFT_NAME)
            .leftFrontMotorName(FRONT_LEFT_NAME)
            .leftFrontMotorDirection(FRONT_LEFT_DIR)
            .leftRearMotorDirection(BACK_LEFT_DIR)
            .rightFrontMotorDirection(FRONT_RIGHT_DIR)
            .rightRearMotorDirection(BACK_RIGHT_DIR);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(PINPOINT_Y_OFFSET_IN)
            .strafePodX(PINPOINT_X_OFFSET_IN)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(ENCODER_RESOLUTION)
            .forwardEncoderDirection(FORWARD_ENCODER_DIRECTION)
            .strafeEncoderDirection(STRAFE_ENCODER_DIRECTION);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99, 300, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
