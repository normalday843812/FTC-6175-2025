package org.firstinspires.ftc.teamcode.auto.testing;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.auto.motion.MotionController;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
import org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig;

@Autonomous(name = "Face Point Bounce Test", group = "Test")
public class FacePointBounceTest extends LinearOpMode {

    private enum State {
        TO_TOP,
        WAIT_TOP,
        TO_BOTTOM,
        WAIT_BOTTOM
    }

    @Override
    public void runOpMode() throws InterruptedException {
        GamepadMap map = new GamepadMap(this);
        Mecanum drive = new Mecanum(this, map);
        drive.init();

        TelemetryHelper tele = new TelemetryHelper(this, AutoUnifiedConfig.TELEMETRY_ENABLED);
        MotionController motion = new MotionController(drive, tele);

        // start pose
        Pose startPose = new Pose(72, 72, 0);
        drive.setStartingPose(startPose);
        drive.startAuto();

        // point to keep facing
        final double FACE_X = 36;  // you said 36,67 at the end
        final double FACE_Y = 67;

        // travel targets
        Pose topPose = new Pose(72, 110, 0);
        Pose bottomPose = new Pose(72, 38, 0);

        State s = State.TO_TOP;
        long waitStartMs = 0;
        final long WAIT_MS = 250;

        waitForStart();
        if (isStopRequested()) return;

        // first move
        motion.followToPoseFacingPoint(topPose, FACE_X, FACE_Y);

        while (opModeIsActive()) {
            drive.operate();

            switch (s) {
                case TO_TOP:
                    if (!motion.isBusy()) {
                        s = State.WAIT_TOP;
                        waitStartMs = System.currentTimeMillis();
                    }
                    break;

                case WAIT_TOP:
                    if (System.currentTimeMillis() - waitStartMs >= WAIT_MS) {
                        motion.followToPoseFacingPoint(bottomPose, FACE_X, FACE_Y);
                        s = State.TO_BOTTOM;
                    }
                    break;

                case TO_BOTTOM:
                    if (!motion.isBusy()) {
                        s = State.WAIT_BOTTOM;
                        waitStartMs = System.currentTimeMillis();
                    }
                    break;

                case WAIT_BOTTOM:
                    if (System.currentTimeMillis() - waitStartMs >= WAIT_MS) {
                        motion.followToPoseFacingPoint(topPose, FACE_X, FACE_Y);
                        s = State.TO_TOP;
                    }
                    break;
            }

            TelemetryHelper.update();
            sleep(20);
        }
    }
}
