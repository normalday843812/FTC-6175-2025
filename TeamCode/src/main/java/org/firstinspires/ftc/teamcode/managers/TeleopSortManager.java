package org.firstinspires.ftc.teamcode.managers;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSlotsColor;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.Timer;

public class TeleopSortManager {
    private final GamepadMap map;
    private final Intake intake;
    private final Spindexer spindexer;
    private final Transfer transfer;
    private final SpindexSlotsColor slots;
    private final InventoryManager inv;
    private boolean isEnabled = true;
    
    private final Timer autoIndexTimer = new Timer();
    private boolean autoIndexing = false;
    private boolean prevSlot0HasBall = false;

    public TeleopSortManager(GamepadMap map, Intake intake, Spindexer spindexer, Transfer transfer,
                             SpindexSlotsColor slots, InventoryManager inv) {
        this.map = map;
        this.intake = intake;
        this.spindexer = spindexer;
        this.transfer = transfer;
        this.slots = slots;
        this.inv = inv;
    }

    public void update() {
        if (!isEnabled) {
            return;
        }

        boolean slot0HasBall = slots.hasAnyBall(0);
        boolean hasEmptySlot = !slots.hasAnyBall(1) || !slots.hasAnyBall(2);
        boolean allSlotsFull = slots.hasAnyBall(0) && slots.hasAnyBall(1) && slots.hasAnyBall(2);
        
        // if all 3 slots are full, turn off intake
        if (allSlotsFull) {
            intake.setAutoMode(Intake.AutoMode.OFF);
        }
        
        // detect when a new ball enters slot 0
        if (slot0HasBall && !prevSlot0HasBall && hasEmptySlot && spindexer.getCurrentSlot() == 0) {
            // start auto-indexing: move to next empty slot
            int nextEmptySlot = findNextEmptySlot();
            if (nextEmptySlot >= 0) {
                spindexer.setSlot(nextEmptySlot);
                autoIndexing = true;
                autoIndexTimer.resetTimer();
            }
        }
        
        prevSlot0HasBall = slot0HasBall;
        
        // handle auto-indexing timer (2 seconds)
        if (autoIndexing) {
            if (autoIndexTimer.getElapsedTimeSeconds() < 2.0) {
                // keep transfer in shooting position with wheels in reverse
                transfer.raiseLever();
                transfer.runTransfer(Transfer.CrState.REVERSE);
            } else {
                // after 2 seconds, return to regular
                autoIndexing = false;
                transfer.lowerLever();
                transfer.runTransfer(Transfer.CrState.OFF);
            }
        }

        // manual controls
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
        
        if (!autoIndexing) {
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
    }
    
    private int findNextEmptySlot() {
        // find the next empty slot (1 or 2)
        if (!slots.hasAnyBall(1)) return 1;
        if (!slots.hasAnyBall(2)) return 2;
        return -1;
    }

    public void setEnabled(boolean enabled) {
        isEnabled = enabled;
    }

    public boolean getEnabled() {
        return isEnabled;
    }
}
