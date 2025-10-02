package org.firstinspires.ftc.teamcode.testing;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name="Limelight Test", group = "Testing")
public class LimelightTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException
    {
        RobotHardware hw = new RobotHardware(this);

        telemetry.setMsTransmissionInterval(11);

        hw.initLimeLight();
        hw.getLimelight().pipelineSwitch(8);

        /*
         * Starts polling for data.
         */
        hw.getLimelight().start();

        while (opModeIsActive()) {
            LLResult result = hw.getLimelight().getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Botpose", botpose.toString());
                }
            }
        }
    }
}
