package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.RgbIndicatorConfig.GREEN_POS;
import static org.firstinspires.ftc.teamcode.config.RgbIndicatorConfig.OFF_POS;
import static org.firstinspires.ftc.teamcode.config.RgbIndicatorConfig.RED_POS;

import com.qualcomm.robotcore.hardware.Servo;

public class RgbIndicator {
    private final Servo rgb;

    public RgbIndicator(Servo rgbIndicator) {
        this.rgb = rgbIndicator;
    }

    public void setGreen() {
        rgb.setPosition(GREEN_POS);
    }

    public void setRed() {
        rgb.setPosition(RED_POS);
    }

    public void setOff() {
        rgb.setPosition(OFF_POS);
    }
}
