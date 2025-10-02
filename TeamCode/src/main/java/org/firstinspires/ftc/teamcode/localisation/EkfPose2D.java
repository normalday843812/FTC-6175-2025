package org.firstinspires.ftc.teamcode.localisation;

import static org.firstinspires.ftc.teamcode.localisation.EkfConstants.*;

import com.bylazar.configurables.annotations.Configurable;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class EkfPose2D {
    // State
    private SimpleMatrix x = new SimpleMatrix(3,1);
    private SimpleMatrix P = SimpleMatrix.identity(3);
    private long lastNanos = -1;

    public void resetTo(double x0, double y0, double h0) {
        x.set(0, x0); x.set(1, y0); x.set(2, wrap(h0));
        P = SimpleMatrix.diag(16.0, 16.0, sq(Math.toRadians(45.0)));
        lastNanos = -1;
    }

    public void predict(double vxR, double vyR, double omega, long nowNanos) {
        double dt = (lastNanos < 0) ? 0.0 : (nowNanos - lastNanos) * 1e-9;
        lastNanos = nowNanos;
        if (dt <= 0) return;

        double h = x.get(2), c = Math.cos(h), s = Math.sin(h);

        // State advance
        double v = dt * (vxR * c - vyR * s);
        x.set(0, x.get(0) + v);
        x.set(1, x.get(1) + dt*( vxR*s + vyR*c ));
        x.set(2, wrap(h + dt*omega));

        // Jacobians
        SimpleMatrix F = new SimpleMatrix(new double[][]{
                {1, 0, dt*(-vxR*s - vyR*c)},
                {0, 1, v},
                {0, 0, 1}
        });
        SimpleMatrix L = new SimpleMatrix(new double[][]{
                {dt*c, -dt*s, 0},
                {dt*s, dt*c, 0},
                {0, 0, dt}
        });
        SimpleMatrix Qu = SimpleMatrix.diag(Q_VX*Q_VX, Q_VY*Q_VY, Q_W*Q_W);
        SimpleMatrix Q = L.mult(Qu).mult(L.transpose());
        P = F.mult(P).mult(F.transpose()).plus(Q);
    }

    public boolean updateFromAprilTag(double zx, double zy, double zh, float decisionMargin) {
        if (decisionMargin < DM_MIN) return false;

        double scale = clamp(DM_REF / Math.max(1.0, decisionMargin), DM_MIN_SCALE, DM_MAX_SCALE);
        SimpleMatrix R = SimpleMatrix.diag(
                sq(scale*R_X_BASE),
                sq(scale*R_Y_BASE),
                sq(scale*R_H_BASE));

        double r0 = zx - x.get(0);
        double r1 = zy - x.get(1);
        double r2 = wrap(zh - x.get(2));
        SimpleMatrix r = new SimpleMatrix(3,1,true, new double[]{r0, r1, r2});

        SimpleMatrix S = P.plus(R);
        SimpleMatrix Sinv = S.invert();
        double d2 = r.transpose().mult(Sinv).mult(r).get(0);
        if (d2 > CHI2_GATE) return false;

        SimpleMatrix K = P.mult(Sinv);
        x = x.plus(K.mult(r));
        x.set(2, wrap(x.get(2)));

        SimpleMatrix I = SimpleMatrix.identity(3);
        P = I.minus(K).mult(P);
        return true;
    }

    public Pose2D toPose2D() {
        return new Pose2D(DistanceUnit.METER, x.get(0), x.get(1), AngleUnit.RADIANS, x.get(2));
    }

    // Helpers
    private static double wrap(double a) { while(a>Math.PI)a-=2*Math.PI; while(a<-Math.PI)a+=2*Math.PI; return a; }
    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
    private static double sq(double v) { return v*v; }
}