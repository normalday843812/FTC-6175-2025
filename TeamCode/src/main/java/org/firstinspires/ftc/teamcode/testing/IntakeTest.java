package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSlotsColor;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

@TeleOp(name = "Intake Test", group = "Testing")
public class IntakeTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hw = new RobotHardware(this);

        GamepadMap map = new GamepadMap(this);

        SpindexSlotsColor slots = new SpindexSlotsColor(
                hw.getSlotColor0(),
                hw.getSlotColor1(),
                hw.getSlotColor2(),
                this
        );

        hw.initIntake();
        Intake intake = new Intake(
                hw.getIntakeMotor(),
                map,
                slots,
                this
        );

        if (isStopRequested()) return;

        waitForStart();

        while (opModeIsActive()) {
            map.update();
            slots.update();
            intake.operate();
            TelemetryHelper.update();
        }
    }
}
