package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public final class Menu {

    public static final class Item {
        public final String title;
        public final String trueName;
        public final String falseName;
        private final BooleanSupplier trueButton;
        private final BooleanSupplier falseButton;
        private final EdgeTrigger trueEdge = new EdgeTrigger();
        private final EdgeTrigger falseEdge = new EdgeTrigger();
        private boolean value;

        public Item(String title,
                    String trueName, BooleanSupplier trueButton,
                    String falseName, BooleanSupplier falseButton,
                    boolean defaultValue) {
            this.title = title;
            this.trueName = trueName;
            this.falseName = falseName;
            this.trueButton = trueButton;
            this.falseButton = falseButton;
            this.value = defaultValue;
        }

        void poll() {
            if (trueEdge.rose(trueButton.getAsBoolean())) value = true;
            if (falseEdge.rose(falseButton.getAsBoolean())) value = false;
        }

        public boolean get() { return value; }
    }

    private final LinearOpMode opmode;
    private final List<Item> items = new ArrayList<>();

    public Menu(LinearOpMode op) { this.opmode = op; }

    public Menu add(Item item) { items.add(item); return this; }

    public void showUntilStart() {
        while (!opmode.isStarted() && !opmode.isStopRequested()) {
            for (Item it : items) it.poll();

            opmode.telemetry.clearAll();
            for (Item it : items) {
                opmode.telemetry.addLine(it.title + ":");
                opmode.telemetry.addLine((it.get() ? "> " : "  ") + it.trueName);
                opmode.telemetry.addLine((!it.get() ? "> " : "  ") + it.falseName);
            }
            opmode.telemetry.addLine("Press start button");
            opmode.telemetry.update();

            opmode.idle();
        }
    }

    public boolean get(String title) {
        for (Item it : items) if (it.title.equals(title)) return it.get();
        return false;
    }
}
