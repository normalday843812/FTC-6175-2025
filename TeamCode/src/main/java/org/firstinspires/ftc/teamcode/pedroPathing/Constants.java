package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Constants {
    public static final FollowerConstants followerConstants = new FollowerConstants();

    public static final PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1,
            1
    );

    public static final String WEBCAM_NAME = "Webcam 1";
    public static final Position CAMERA_POS =
            new Position(DistanceUnit.INCH, -2.0, 6.0, 8.0, 0);

    public static final YawPitchRollAngles CAMERA_ORIENT =
            new YawPitchRollAngles(AngleUnit.RADIANS, 0, -Math.PI / 2, 0, 0);

    public static final Pose START_POSE = new Pose(0, 0, 0);

    public static final PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-5)
            .strafePodX(0.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        Localizer fusedLocalizer = new WebcamPinpointLocaliser(
                hardwareMap,
                localizerConstants,
                START_POSE,
                WEBCAM_NAME,
                CAMERA_POS,
                CAMERA_ORIENT
        );

        PedroMecanumDrivetrain drivetrain = new PedroMecanumDrivetrain(
                hardwareMap,
                "front_right_drive", "front_left_drive", "back_right_drive", "back_left_drive"
        );

        return new Follower(
                followerConstants,
                fusedLocalizer,
                drivetrain,
                pathConstraints
        );
    }
}
