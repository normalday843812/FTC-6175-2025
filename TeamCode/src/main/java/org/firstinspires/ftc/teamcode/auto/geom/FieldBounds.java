package org.firstinspires.ftc.teamcode.auto.geom;

import com.pedropathing.geometry.Pose;

public final class FieldBounds {
    private FieldBounds() {}

    // Pedro coordinates
    public static Pose clampToShootingRegion(Pose want) {
        double x = want.getX();
        double y = want.getY();

        y = Math.min(y, 144.0 - 1.0);
        if (y <= x) y = x + 1.0;
        if (y <= -x + 144.0) y = -x + 144.0 + 1.0;

        return new Pose(x, y, want.getHeading());
    }
}