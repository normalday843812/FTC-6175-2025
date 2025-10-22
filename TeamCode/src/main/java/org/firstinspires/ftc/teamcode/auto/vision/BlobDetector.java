package org.firstinspires.ftc.teamcode.auto.vision;

import static com.pedropathing.math.MathFunctions.clamp;

import android.util.Size;

import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.Comparator;
import java.util.List;

public class BlobDetector {
    public static class Blob {
        public final boolean seen;
        public final double cxNorm;
        public final double cyNorm;
        public final double radiusNorm;
        public final int area;

        public Blob(boolean seen, double cxNorm, double cyNorm, double radiusNorm, int area) {
            this.seen = seen;
            this.cxNorm = cxNorm;
            this.cyNorm = cyNorm;
            this.radiusNorm = radiusNorm;
            this.area = area;
        }

        public static Blob none() {
            return new Blob(false, 0, 0, 0, 0);
        }
    }

    private final ColorBlobLocatorProcessor purple;
    private final ColorBlobLocatorProcessor green;
    private final int frameW;
    private final int frameH;

    public BlobDetector(ColorBlobLocatorProcessor purple, ColorBlobLocatorProcessor green, Size frameSize) {
        this.purple = purple;
        this.green = green;
        this.frameW = frameSize.getWidth();
        this.frameH = frameSize.getHeight();
    }

    public Blob bestPurpleBlob() {
        return bestFromProcessor(purple);
    }

    public Blob bestGreenBlob() {
        return bestFromProcessor(green);
    }

    /** @noinspection unused*/
    public boolean anyBlobVisible() {
        return bestPurpleBlob().seen || bestGreenBlob().seen;
    }

    public Blob pickAccordingToNeed(boolean needPurple) {
        Blob pb = bestPurpleBlob();
        Blob gb = bestGreenBlob();
        if (needPurple) {
            if (pb.seen) return pb;
            if (gb.seen) return gb;
        } else {
            if (gb.seen) return gb;
            if (pb.seen) return pb;
        }
        return Blob.none();
    }

    private Blob bestFromProcessor(ColorBlobLocatorProcessor p) {
        List<ColorBlobLocatorProcessor.Blob> blobs = p.getBlobs();
        if (blobs == null || blobs.isEmpty()) return Blob.none();

        ColorBlobLocatorProcessor.Blob best = blobs.stream()
                .max(Comparator.comparingDouble(b -> b.getCircle().getRadius()))
                .orElse(blobs.get(0));

        Circle c = best.getCircle();
        double cxNorm = (c.getX() - frameW * 0.5) / (frameW * 0.5);
        double cyNorm = ((frameH * 0.5) - c.getY()) / (frameH * 0.5);
        double radiusNorm = c.getRadius() / Math.min(frameW, frameH);
        int area = (int) Math.round(Math.PI * c.getRadius() * c.getRadius());

        return new Blob(true, clamp(cxNorm, -1, 1), clamp(cyNorm, -1, 1), clamp(radiusNorm, 0, 1), area);
    }
}