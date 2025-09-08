package org.firstinspires.ftc.teamcode.misc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Play Thick of It", group = "TeleOp")
public class ThickOfItOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ThickOfIt thick = new ThickOfIt(/* totalSounds= */33, /* defaultDurationMs= */5000);

        int loaded = thick.preloadSounds();
        telemetry.addData("Init", "Preloaded %d / %d", loaded, thick.getTotalSounds());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            thick.release();
            return;
        }

        // Optional: loop the playlist. Remove if you want one-shot behavior.
        thick.setLoop(false);

        // Start at track 1; you can also do thick.startAt(10) to begin at “11.mp3”
        thick.start();

        while (opModeIsActive() && !isStopRequested()) {
            thick.update();

            telemetry.addData("Playing", thick.isPlaying());
            telemetry.addData("Track", thick.getCurrentTrackNumber());
            telemetry.update();

            // Be kind to the scheduler / CPU on the RC phone.
            sleep(10);
        }

        // Ensure resources are released even if user stops early
        try {
            thick.stop();
        } finally {
            thick.release();
        }
    }
}
