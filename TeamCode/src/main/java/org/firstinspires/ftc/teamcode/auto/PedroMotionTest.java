package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.config.DecodeGameConfig;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
import org.firstinspires.ftc.teamcode.util.Timer;

@Autonomous(name = "Pedro Motion Test", group = "Tuning")
@Configurable
public class PedroMotionTest extends LinearOpMode {

    // Alliance/side config
    public static boolean IS_RED = true;
    public static boolean IS_AUDIENCE_SIDE = true;

    // Which movements to run
    public static boolean DO_PATH_TO_SHOOT = true;
    public static boolean DO_INTAKE_SET_0 = true;
    public static boolean DO_INTAKE_SET_1 = false;
    public static boolean DO_INTAKE_SET_2 = false;
    public static boolean DO_FINAL_MOVE = true;

    // Timing
    public static double PATH_TIMEOUT_S = 5.0;
    public static double WAIT_AT_POSE_S = 1.0;  // Pause at each destination

    // Creep test (simulates INTAKE_FORWARD)
    public static boolean DO_CREEP_TEST = false;
    public static double CREEP_SPEED = 0.25;
    public static double CREEP_TIME_S = 2.0;

    private enum State {
        PATH_TO_SHOOT,
        WAIT_AT_SHOOT,
        PATH_TO_INTAKE_0,
        CREEP_0,
        WAIT_AT_INTAKE_0,
        PATH_TO_INTAKE_1,
        CREEP_1,
        WAIT_AT_INTAKE_1,
        PATH_TO_INTAKE_2,
        CREEP_2,
        WAIT_AT_INTAKE_2,
        PATH_TO_FINAL,
        WAIT_AT_FINAL,
        DONE
    }

    @Override
    public void runOpMode() throws InterruptedException {
        GamepadMap map = new GamepadMap(this);
        Mecanum drive = new Mecanum(this, map);
        drive.init();

        Follower follower = drive.getFollower();
        TelemetryHelper tele = new TelemetryHelper(this, true);
        Timer timer = new Timer();

        // Get poses from config
        Pose startPose = DecodeGameConfig.startPose(IS_RED, IS_AUDIENCE_SIDE);
        Pose shootPose = DecodeGameConfig.shootPose(IS_RED);
        Pose finalPose = DecodeGameConfig.finalPose(IS_RED);
        Pose[] intakeSets = IS_RED ? DecodeGameConfig.INTAKE_SETS_RED : DecodeGameConfig.INTAKE_SETS_BLUE;

        drive.setStartingPose(startPose);

        telemetry.addLine("Pedro Motion Test Ready");
        telemetry.addData("Alliance", IS_RED ? "RED" : "BLUE");
        telemetry.addData("Side", IS_AUDIENCE_SIDE ? "AUDIENCE" : "DEPOT");
        telemetry.addData("Start", "(%.1f, %.1f, %.1f°)", startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));
        telemetry.addData("Shoot", "(%.1f, %.1f, %.1f°)", shootPose.getX(), shootPose.getY(), Math.toDegrees(shootPose.getHeading()));
        telemetry.addData("Final", "(%.1f, %.1f, %.1f°)", finalPose.getX(), finalPose.getY(), Math.toDegrees(finalPose.getHeading()));
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        drive.startAuto();

        // Determine starting state
        State state;
        if (DO_PATH_TO_SHOOT) {
            state = State.PATH_TO_SHOOT;
        } else if (DO_INTAKE_SET_0) {
            state = State.PATH_TO_INTAKE_0;
        } else if (DO_INTAKE_SET_1) {
            state = State.PATH_TO_INTAKE_1;
        } else if (DO_INTAKE_SET_2) {
            state = State.PATH_TO_INTAKE_2;
        } else if (DO_FINAL_MOVE) {
            state = State.PATH_TO_FINAL;
        } else {
            state = State.DONE;
        }

        boolean pathIssued = false;
        timer.resetTimer();

        while (opModeIsActive()) {
            Pose current = follower.getPose();

            switch (state) {
                case PATH_TO_SHOOT:
                    if (!pathIssued) {
                        followToPose(follower, shootPose);
                        pathIssued = true;
                        timer.resetTimer();
                    }
                    if (!follower.isBusy() || timer.getElapsedTimeSeconds() >= PATH_TIMEOUT_S) {
                        state = State.WAIT_AT_SHOOT;
                        pathIssued = false;
                        timer.resetTimer();
                    }
                    break;

                case WAIT_AT_SHOOT:
                    drive.setAutoDrive(0, 0, 0, true, 0);
                    if (timer.getElapsedTimeSeconds() >= WAIT_AT_POSE_S) {
                        if (DO_INTAKE_SET_0) {
                            state = State.PATH_TO_INTAKE_0;
                        } else if (DO_INTAKE_SET_1) {
                            state = State.PATH_TO_INTAKE_1;
                        } else if (DO_INTAKE_SET_2) {
                            state = State.PATH_TO_INTAKE_2;
                        } else if (DO_FINAL_MOVE) {
                            state = State.PATH_TO_FINAL;
                        } else {
                            state = State.DONE;
                        }
                        timer.resetTimer();
                    }
                    break;

                case PATH_TO_INTAKE_0:
                    if (!pathIssued) {
                        followToPose(follower, intakeSets[0]);
                        pathIssued = true;
                        timer.resetTimer();
                    }
                    if (!follower.isBusy() || timer.getElapsedTimeSeconds() >= PATH_TIMEOUT_S) {
                        state = DO_CREEP_TEST ? State.CREEP_0 : State.WAIT_AT_INTAKE_0;
                        pathIssued = false;
                        timer.resetTimer();
                    }
                    break;

                case CREEP_0:
                    drive.setAutoDrive(0, CREEP_SPEED, 0, true, 0);
                    if (timer.getElapsedTimeSeconds() >= CREEP_TIME_S) {
                        drive.clearAutoCommand();
                        state = State.WAIT_AT_INTAKE_0;
                        timer.resetTimer();
                    }
                    break;

                case WAIT_AT_INTAKE_0:
                    drive.setAutoDrive(0, 0, 0, true, 0);
                    if (timer.getElapsedTimeSeconds() >= WAIT_AT_POSE_S) {
                        if (DO_PATH_TO_SHOOT) {
                            state = State.PATH_TO_SHOOT;
                        } else if (DO_INTAKE_SET_1) {
                            state = State.PATH_TO_INTAKE_1;
                        } else if (DO_INTAKE_SET_2) {
                            state = State.PATH_TO_INTAKE_2;
                        } else if (DO_FINAL_MOVE) {
                            state = State.PATH_TO_FINAL;
                        } else {
                            state = State.DONE;
                        }
                        timer.resetTimer();
                    }
                    break;

                case PATH_TO_INTAKE_1:
                    if (!pathIssued) {
                        followToPose(follower, intakeSets[1]);
                        pathIssued = true;
                        timer.resetTimer();
                    }
                    if (!follower.isBusy() || timer.getElapsedTimeSeconds() >= PATH_TIMEOUT_S) {
                        state = DO_CREEP_TEST ? State.CREEP_1 : State.WAIT_AT_INTAKE_1;
                        pathIssued = false;
                        timer.resetTimer();
                    }
                    break;

                case CREEP_1:
                    drive.setAutoDrive(0, CREEP_SPEED, 0, true, 0);
                    if (timer.getElapsedTimeSeconds() >= CREEP_TIME_S) {
                        drive.clearAutoCommand();
                        state = State.WAIT_AT_INTAKE_1;
                        timer.resetTimer();
                    }
                    break;

                case WAIT_AT_INTAKE_1:
                    drive.setAutoDrive(0, 0, 0, true, 0);
                    if (timer.getElapsedTimeSeconds() >= WAIT_AT_POSE_S) {
                        if (DO_PATH_TO_SHOOT) {
                            state = State.PATH_TO_SHOOT;
                        } else if (DO_INTAKE_SET_2) {
                            state = State.PATH_TO_INTAKE_2;
                        } else if (DO_FINAL_MOVE) {
                            state = State.PATH_TO_FINAL;
                        } else {
                            state = State.DONE;
                        }
                        timer.resetTimer();
                    }
                    break;

                case PATH_TO_INTAKE_2:
                    if (!pathIssued) {
                        followToPose(follower, intakeSets[2]);
                        pathIssued = true;
                        timer.resetTimer();
                    }
                    if (!follower.isBusy() || timer.getElapsedTimeSeconds() >= PATH_TIMEOUT_S) {
                        state = DO_CREEP_TEST ? State.CREEP_2 : State.WAIT_AT_INTAKE_2;
                        pathIssued = false;
                        timer.resetTimer();
                    }
                    break;

                case CREEP_2:
                    drive.setAutoDrive(0, CREEP_SPEED, 0, true, 0);
                    if (timer.getElapsedTimeSeconds() >= CREEP_TIME_S) {
                        drive.clearAutoCommand();
                        state = State.WAIT_AT_INTAKE_2;
                        timer.resetTimer();
                    }
                    break;

                case WAIT_AT_INTAKE_2:
                    drive.setAutoDrive(0, 0, 0, true, 0);
                    if (timer.getElapsedTimeSeconds() >= WAIT_AT_POSE_S) {
                        if (DO_PATH_TO_SHOOT) {
                            state = State.PATH_TO_SHOOT;
                        } else if (DO_FINAL_MOVE) {
                            state = State.PATH_TO_FINAL;
                        } else {
                            state = State.DONE;
                        }
                        timer.resetTimer();
                    }
                    break;

                case PATH_TO_FINAL:
                    if (!pathIssued) {
                        followToPose(follower, finalPose);
                        pathIssued = true;
                        timer.resetTimer();
                    }
                    if (!follower.isBusy() || timer.getElapsedTimeSeconds() >= PATH_TIMEOUT_S) {
                        state = State.WAIT_AT_FINAL;
                        pathIssued = false;
                        timer.resetTimer();
                    }
                    break;

                case WAIT_AT_FINAL:
                    drive.setAutoDrive(0, 0, 0, true, 0);
                    if (timer.getElapsedTimeSeconds() >= WAIT_AT_POSE_S) {
                        state = State.DONE;
                    }
                    break;

                case DONE:
                    drive.setAutoDrive(0, 0, 0, true, 0);
                    break;
            }

            follower.update();

            // Telemetry
            tele.addLine("=== PEDRO MOTION TEST ===")
                    .addData("State", state.name())
                    .addData("Timer", "%.2f s", timer.getElapsedTimeSeconds())
                    .addData("Busy", "%b", follower.isBusy())
                    .addData("Current", "(%.1f, %.1f, %.1f°)",
                            current.getX(), current.getY(), Math.toDegrees(current.getHeading()));

            // Show target based on current state
            Pose target = getTargetForState(state, shootPose, finalPose, intakeSets);
            if (target != null) {
                double dist = Math.hypot(target.getX() - current.getX(), target.getY() - current.getY());
                tele.addData("Target", "(%.1f, %.1f, %.1f°)",
                                target.getX(), target.getY(), Math.toDegrees(target.getHeading()))
                        .addData("Distance", "%.2f in", dist);
            }

            TelemetryHelper.update();
            sleep(20);
        }
    }

    private void followToPose(Follower follower, Pose target) {
        Pose current = follower.getPose();

        double midX = (current.getX() + target.getX()) / 2.0;
        double midY = (current.getY() + target.getY()) / 2.0;
        Pose control = new Pose(midX, midY, target.getHeading());

        PathChain chain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        target,
                        control,
                        target))
                .setLinearHeadingInterpolation(current.getHeading(), target.getHeading())
                .build();

        follower.followPath(chain);
    }

    private Pose getTargetForState(State state, Pose shootPose, Pose finalPose, Pose[] intakeSets) {
        switch (state) {
            case PATH_TO_SHOOT:
            case WAIT_AT_SHOOT:
                return shootPose;
            case PATH_TO_INTAKE_0:
            case CREEP_0:
            case WAIT_AT_INTAKE_0:
                return intakeSets[0];
            case PATH_TO_INTAKE_1:
            case CREEP_1:
            case WAIT_AT_INTAKE_1:
                return intakeSets[1];
            case PATH_TO_INTAKE_2:
            case CREEP_2:
            case WAIT_AT_INTAKE_2:
                return intakeSets[2];
            case PATH_TO_FINAL:
            case WAIT_AT_FINAL:
                return finalPose;
            default:
                return null;
        }
    }
}