package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.config.AutoConfig.START_BLUE_AUDIENCE;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.START_BLUE_DEPOT;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.START_RED_AUDIENCE;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.START_RED_DEPOT;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Menu;

@Autonomous(name = "Auto test", group = "Pedro")
public class AutoTest extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    @Override
    public void runOpMode() throws InterruptedException {
        Menu menu = new Menu(this)
                .add(new Menu.Item(
                        "Alliance",
                        "Red (B)", () -> gamepad1.b,
                        "Blue (X)", () -> gamepad1.x,
                        true))
                .add(new Menu.Item(
                        "Side",
                        "Audience", () -> gamepad1.y,
                        "Depot", () -> gamepad1.a,
                        true
                ))
                .add(new Menu.Item(
                        "Actively intake?",
                        "Yes (A)", () -> gamepad1.a,
                        "No (B)", () -> gamepad1.b,
                        true))
                .add(new Menu.Item(
                        "Shoot Preloaded?",
                        "Yes (A)", () -> gamepad1.a,
                        "No (B)", () -> gamepad1.b,
                        true
                ))
                .add(new Menu.Item(
                        "Move?",
                        "Yes (A)", () -> gamepad1.a,
                        "No (B)", () -> gamepad1.b,
                        true
                ));

        menu.showUntilStart();

        boolean isRed = menu.get("Alliance");
        boolean isAudienceSide = menu.get("Side");
        boolean activelyIntake = menu.get("Actively intake?");
        boolean shootPreloaded = menu.get("Shoot Preloaded?");
        boolean move = menu.get("Move?");

        telemetry.addData("Alliance", isRed ? "RED" : "BLUE");
        telemetry.addData("Side", isAudienceSide ? "AUDIENCE" : "DEPOT");
        telemetry.addData("Actively Intake", activelyIntake ? "YES" : "NO");
        if ((activelyIntake || shootPreloaded) && !move) {
            move = true;
            String result;
            if (activelyIntake && shootPreloaded) {
                result = "Actively Intake and Shoot Preloaded require movement!";
            } else if (activelyIntake) {
                result = "Actively Intake requires movement!";
            } else {
                result = "Shoot Preloaded requires movement!";
            }
            telemetry.addLine(result);
            telemetry.addData("Move", "YES");
        } else {
            telemetry.addData("Move", move ? "YES" : "NO");
        }
        telemetry.update();

        Pose startPose;
        if (isRed) {
            if (isAudienceSide) {
                startPose = START_RED_AUDIENCE;
            } else {
                startPose = START_RED_DEPOT;
            }
        } else {
            if (isAudienceSide) {
                startPose = START_BLUE_AUDIENCE;
            } else {
                startPose = START_BLUE_DEPOT;
            }
        }

        if (isStopRequested()) return;
        waitForStart();

        while (opModeIsActive()) {
            sleep(20);
        }
    }
}