package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.sensors.ColorClassifier;

@TeleOp(name = "Color Sensor Test", group = "Testing")
public class ColorSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware hw = new RobotHardware(this);
        hw.initSpindexColorSensors();

        NormalizedColorSensor sensor0 = hw.getSlotColor0();
        NormalizedColorSensor sensor1 = hw.getSlotColor1();
        NormalizedColorSensor sensor2 = hw.getSlotColor2();

        ColorClassifier classifier0 = new ColorClassifier();
        ColorClassifier classifier1 = new ColorClassifier();
        ColorClassifier classifier2 = new ColorClassifier();

        // Individual gains for each sensor
        float gain0 = 100f;
        float gain1 = 100f;
        float gain2 = 100f;

        // Track which sensor is selected for adjustment
        int selectedSensor = -1; // -1 = none, 0/1/2 = sensor number

        waitForStart();

        while (opModeIsActive()) {
            // Select sensor to adjust with Y/B/X
            if (gamepad1.y) selectedSensor = 0;
            if (gamepad1.b) selectedSensor = 1;
            if (gamepad1.x) selectedSensor = 2;
            if (gamepad1.a) selectedSensor = -1; // Press A to deselect

            // Adjust gain for selected sensor with D-pad
            if (selectedSensor != -1) {
                float adjustment = 0;
                if (gamepad1.dpad_up) adjustment = 5f;
                if (gamepad1.dpad_down) adjustment = -5f;
                if (gamepad1.dpad_right) adjustment = 1f;
                if (gamepad1.dpad_left) adjustment = -1f;

                if (selectedSensor == 0) {
                    gain0 = Math.max(1f, Math.min(255f, gain0 + adjustment));
                } else if (selectedSensor == 1) {
                    gain1 = Math.max(1f, Math.min(255f, gain1 + adjustment));
                } else if (selectedSensor == 2) {
                    gain2 = Math.max(1f, Math.min(255f, gain2 + adjustment));
                }
            }

            // Display header
            telemetry.addLine("=== COLOR SENSOR TEST ===");
            telemetry.addLine("Press Y/B/X to select sensor 0/1/2, A to deselect");
            telemetry.addLine("D-pad: Up/Down = ±5, Right/Left = ±1");
            telemetry.addData("Selected", selectedSensor == -1 ? "None" : "Sensor " + selectedSensor);
            telemetry.addLine();

            // Read and display each sensor
            readAndDisplaySensor(sensor0, classifier0, 0, gain0, selectedSensor == 0);
            telemetry.addLine();
            readAndDisplaySensor(sensor1, classifier1, 1, gain1, selectedSensor == 1);
            telemetry.addLine();
            readAndDisplaySensor(sensor2, classifier2, 2, gain2, selectedSensor == 2);

            telemetry.update();
        }
    }

    private void readAndDisplaySensor(NormalizedColorSensor sensor,
                                      ColorClassifier classifier,
                                      int slotNum,
                                      float gain,
                                      boolean isSelected) {
        try {
            sensor.setGain(gain);
            NormalizedRGBA rgba = sensor.getNormalizedColors();

            // Convert to 8-bit RGB
            int r = Math.round(rgba.red * 255f);
            int g = Math.round(rgba.green * 255f);
            int b = Math.round(rgba.blue * 255f);
            int a = Math.round(rgba.alpha * 255f);

            // Convert to HSV for better understanding
            float[] hsv = new float[3];
            android.graphics.Color.RGBToHSV(r, g, b, hsv);

            // Feed to classifier
            classifier.pushSample(r, g, b, rgba.alpha);

            // Determine detected color
            String detected = "NONE";
            if (classifier.isPurple()) detected = "PURPLE";
            else if (classifier.isGreen()) detected = "GREEN";

            // Display all info
            String header = isSelected ? ">>> SLOT " + slotNum + " <<<" : "--- SLOT " + slotNum + " ---";
            telemetry.addLine(header);
            telemetry.addData("Gain", "%.0f %s", gain, isSelected ? "(SELECTED)" : "");
            telemetry.addData("RGB", "(%d, %d, %d)", r, g, b);
            telemetry.addData("Alpha", "%d", a);
            telemetry.addData("HSV", "(H:%.0f° S:%.2f V:%.2f)", hsv[0], hsv[1], hsv[2]);
            telemetry.addData("DETECTED", detected);

            // Visual color indicator
            String colorBar = getColorBar(r, g, b);
            telemetry.addData("Visual", colorBar);

            // Show if sensor is dead (all zeros)
            if (r == 0 && g == 0 && b == 0) {
                telemetry.addLine("*** SENSOR DEAD OR BLOCKED ***");
            }

        } catch (Exception e) {
            telemetry.addLine("--- SLOT " + slotNum + " ERROR ---");
            telemetry.addData("Error", e.getMessage());
        }
    }

    private String getColorBar(int r, int g, int b) {
        // Create a simple visual representation
        int max = Math.max(Math.max(r, g), b);
        if (max == 0) return "[DEAD]";

        int rBar = (r * 5) / max;
        int gBar = (g * 5) / max;
        int bBar = (b * 5) / max;

        return "R:" + repeat("■", rBar) + " G:" + repeat("■", gBar) + " B:" + repeat("■", bBar);
    }

    private String repeat(String s, int n) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < n; i++) sb.append(s);
        return sb.toString();
    }
}
