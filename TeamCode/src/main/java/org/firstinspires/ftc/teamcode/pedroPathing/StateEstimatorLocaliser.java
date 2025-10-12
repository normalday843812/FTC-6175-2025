package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;

public class StateEstimatorLocaliser implements Localizer {

    public StateEstimatorLocaliser() {
    }
    /**
     * This returns the current pose estimate from the Localizer.
     *
     * @return returns the pose as a Pose object.
     */
    @Override
    public Pose getPose() {
        return null;
    }

    /**
     * This returns the current velocity estimate from the Localizer.
     *
     * @return returns the velocity as a Pose object.
     */
    @Override
    public Pose getVelocity() {
        return null;
    }

    /**
     * This returns the current velocity estimate from the Localizer as a Vector.
     *
     * @return returns the velocity as a Vector.
     */
    @Override
    public Vector getVelocityVector() {
        return null;
    }

    /**
     * This sets the start pose of the Localizer. Changing the start pose should move the robot as if
     * all its previous movements were displacing it from its new start pose.
     *
     * @param setStart the new start pose
     */
    @Override
    public void setStartPose(Pose setStart) {

    }

    /**
     * This sets the current pose estimate of the Localizer. Changing this should just change the
     * robot's current pose estimate, not anything to do with the start pose.
     *
     * @param setPose the new current pose estimate
     */
    @Override
    public void setPose(Pose setPose) {

    }

    /**
     * This calls an update to the Localizer, updating the current pose estimate and current velocity
     * estimate.
     */
    @Override
    public void update() {

    }

    /**
     * This returns how far the robot has turned in radians, in a number not clamped between 0 and
     * 2 * pi radians. This is used for some tuning things and nothing actually within the following.
     *
     * @return returns how far the robot has turned in total, in radians.
     */
    @Override
    public double getTotalHeading() {
        return 0;
    }

    /**
     * This returns the multiplier applied to forward movement measurement to convert from encoder
     * ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the forward ticks to inches multiplier
     */
    @Override
    public double getForwardMultiplier() {
        return 0;
    }

    /**
     * This returns the multiplier applied to lateral/strafe movement measurement to convert from
     * encoder ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the lateral/strafe ticks to inches multiplier
     */
    @Override
    public double getLateralMultiplier() {
        return 0;
    }

    /**
     * This returns the multiplier applied to turning movement measurement to convert from encoder
     * ticks to radians. This is found empirically through a tuner.
     *
     * @return returns the turning ticks to radians multiplier
     */
    @Override
    public double getTurningMultiplier() {
        return 0;
    }

    /**
     * This resets the IMU of the localizer, if applicable.
     */
    @Override
    public void resetIMU() throws InterruptedException {

    }

    /**
     * This is overridden to return the IMU's heading estimate, if there is one.
     *
     * @return returns the IMU's heading estimate if it exists
     */
    @Override
    public double getIMUHeading() {
        return 0;
    }

    /**
     * This returns whether if any component of robot's position is NaN.
     *
     * @return returns if any component of the robot's position is NaN
     */
    @Override
    public boolean isNAN() {
        return false;
    }
}
