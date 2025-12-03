package org.firstinspires.ftc.teamcode.auto.motion;

import static com.pedropathing.math.MathFunctions.clamp;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.HEADING_DEADBAND_DEG;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.HEADING_MAX_ROT;

import com.pedropathing.control.PIDFController;
import com.pedropathing.control.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;

public class HeadingController {
    private PIDFCoefficients primaryCoefficients;
    private PIDFCoefficients secondaryCoefficients;

    private PIDFController primaryController;
    private PIDFController secondaryController;

    private double switchThresholdRad = 0.0;
    private boolean secondaryEnabled = false;

    public HeadingController() {
        reloadCoefficients();
    }

    public double update(double desiredDeg, double currentDeg) {
        double errorDeg = wrapDeg(desiredDeg - currentDeg);

        if (Math.abs(errorDeg) < HEADING_DEADBAND_DEG) {
            return 0.0;
        }

        double errorRad = Math.toRadians(errorDeg);
        PIDFController controller = selectController(errorRad);

        controller.updateError(errorRad);
        double output = controller.run();

        return clamp(output, -HEADING_MAX_ROT, HEADING_MAX_ROT);
    }

    public void reset() {
        primaryController = new PIDFController(primaryCoefficients);
        if (secondaryEnabled && secondaryCoefficients != null) {
            secondaryController = new PIDFController(secondaryCoefficients);
        } else {
            secondaryController = null;
        }
    }

    public void reloadCoefficients() {
        primaryCoefficients = Constants.followerConstants.getCoefficientsHeadingPIDF();
        primaryController = new PIDFController(primaryCoefficients);

        secondaryEnabled = Constants.followerConstants.isUseSecondaryHeadingPIDF();
        if (secondaryEnabled) {
            secondaryCoefficients = Constants.followerConstants.getCoefficientsSecondaryHeadingPIDF();
            secondaryController = new PIDFController(secondaryCoefficients);
            switchThresholdRad = Constants.followerConstants.getHeadingPIDFSwitch();
        } else {
            secondaryCoefficients = null;
            secondaryController = null;
            switchThresholdRad = 0.0;
        }
    }

    private PIDFController selectController(double errorRad) {
        if (secondaryEnabled
                && secondaryController != null
                && switchThresholdRad > 0
                && Math.abs(errorRad) <= switchThresholdRad) {
            return secondaryController;
        }
        return primaryController;
    }

    private static double wrapDeg(double angleDeg) {
        while (angleDeg > 180) angleDeg -= 360;
        while (angleDeg < -180) angleDeg += 360;
        return angleDeg;
    }
}