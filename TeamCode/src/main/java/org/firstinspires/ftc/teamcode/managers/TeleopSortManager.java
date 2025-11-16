package org.firstinspires.ftc.teamcode.managers;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.SlotColorSensors;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

public class TeleopSortManager {
    private final GamepadMap map;
    private final Intake intake;
    private final Spindexer spindexer;
    private final Transfer transfer;
    private final SlotColorSensors slots;
    private final InventoryManager inv;
    private boolean isEnabled = true;

    public TeleopSortManager(GamepadMap map, Intake intake, Spindexer spindexer, Transfer transfer,
                             SlotColorSensors slots, InventoryManager inv) {
        this.map = map;
        this.intake = intake;
        this.spindexer = spindexer;
        this.transfer = transfer;
        this.slots = slots;
        this.inv = inv;
    }

    public void update() {
        if (isEnabled) {
            return;
        }

        int slot;
        if (map.findGreenBall) {
            inv.setWantPurple(false);
            slot = inv.decideTargetSlot(slots, spindexer);
            if (slot >= 0 && slot != spindexer.getCurrentSlot()) spindexer.setSlot(slot);
        } else if (map.findPurpleBall) {
            inv.setWantPurple(true);
            slot = inv.decideTargetSlot(slots, spindexer);
            if (slot >= 0 && slot != spindexer.getCurrentSlot()) spindexer.setSlot(slot);
        }
        if (slots.hasAnyBall(0)) {
            transfer.raiseLever();
        } else {
            transfer.lowerLever();
            transfer.runTransfer(Transfer.CrState.REVERSE);
        }

        if (map.transferButton) {
            transfer.runTransfer(Transfer.CrState.FORWARD);
            transfer.flick();
        } else {
            if (slots.isFull()) {
                intake.setAutoMode(Intake.AutoMode.OFF);
            } else {
                intake.setAutoMode(Intake.AutoMode.REVERSE);
                spindexer.setSlot(inv.findNearestEmptySlot(slots, spindexer));
            }
        }
    }

    public void setEnabled(boolean enabled) {
        isEnabled = enabled;
    }

    public boolean getEnabled() {
        return isEnabled;
    }
}
