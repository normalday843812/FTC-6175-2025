package org.firstinspires.ftc.teamcode.util;

public final class EdgeTrigger {
    private boolean last = false;

    public boolean rose(boolean now) {
        boolean r = now && !last;
        last = now;
        return r;
    }
}
