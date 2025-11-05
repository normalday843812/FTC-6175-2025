package org.firstinspires.ftc.teamcode.auto.geom;

import static com.pedropathing.math.MathFunctions.clamp;
import static org.firstinspires.ftc.teamcode.config.DecodeGameConfig.MAXIMUM_BLUE_X;
import static org.firstinspires.ftc.teamcode.config.DecodeGameConfig.MINIMUM_BLUE_X;
import static org.firstinspires.ftc.teamcode.config.DecodeGameConfig.MINIMUM_RED_X;
import static org.firstinspires.ftc.teamcode.config.DecodeGameConfig.MAXIMUM_RED_X;
import static org.firstinspires.ftc.teamcode.config.DecodeGameConfig.MINIMUM_Y;
import static org.firstinspires.ftc.teamcode.config.DecodeGameConfig.MAXIMUM_Y;

import com.pedropathing.geometry.Pose;

public final class FieldBounds {
    private FieldBounds() {
    }
    // Pedro coordinates
    public static Pose clampToAllianceRect(Pose want, boolean isRed) {
        double minX = isRed ? MINIMUM_RED_X : MINIMUM_BLUE_X;
        double maxX = isRed ? MAXIMUM_RED_X : MAXIMUM_BLUE_X;
        double x = clamp(want.getX(), minX, maxX);
        double y = clamp(want.getY(), MINIMUM_Y, MAXIMUM_Y);
        return new Pose(x, y, want.getHeading());
    }

    public static Pose clampToShootingRegionTri(Pose want) {
        double x = want.getX();
        double y = want.getY();
        y = Math.min(y, 144.0 - 1.0);
        if (y <= x) y = x + 1.0;
        if (y <= -x + 144.0) y = -x + 144.0 + 1.0;
        return new Pose(x, y, want.getHeading());
    }
}
