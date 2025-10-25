package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

@TeleOp(name = "Transfer Test", group = "Testing")
public class TransferTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hw = new RobotHardware(this);

        GamepadMap map = new GamepadMap(this);

        hw.initTransfer();
        Transfer transfer = new Transfer(hw.getTransferServo(), map, this);

        if (isStopRequested()) return;

        waitForStart();

        while (opModeIsActive()) {
            map.update();
            transfer.operate();
            TelemetryHelper.update();
        }
    }
}
