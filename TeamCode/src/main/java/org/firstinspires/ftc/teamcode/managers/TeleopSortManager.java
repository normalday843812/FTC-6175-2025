package org.firstinspires.ftc.teamcode.managers;

import static org.firstinspires.ftc.teamcode.config.TeleopSortManagerConfig.BALL_GRIP_DELAY_S;
import static org.firstinspires.ftc.teamcode.config.TeleopSortManagerConfig.MANUAL_SPINDEX_COOLDOWN_S;
import static org.firstinspires.ftc.teamcode.config.TeleopSortManagerConfig.TRANSFER_LOWER_DELAY_S;

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
    private final Timer ballGripTimer = new Timer();
    private final Timer manualSpindexCooldown = new Timer();
    private boolean autoIndexing = false;
    private boolean waitingToSpindex = false;
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
        
        // detect manual spindexer controls (X/Y for ball finding, gamepad2 X/Y for manual stepping)
        boolean manualSpindexControl = map.findGreenBall || map.findPurpleBall || 
                                        map.spindexerForward || map.spindexerBackward;
        if (manualSpindexControl) {
            manualSpindexCooldown.resetTimer();
        }
        
        // if all 3 slots are full, turn off intake
        if (allSlotsFull) {
            intake.setAutoMode(Intake.AutoMode.OFF);
        }
        
        // detect when color sensor sees a new ball (but not if recently manually controlled)
        boolean cooldownExpired = manualSpindexCooldown.getElapsedTimeSeconds() > MANUAL_SPINDEX_COOLDOWN_S;
        if (slot0HasBall && !prevSlot0HasBall && !allSlotsFull && !waitingToSpindex && !autoIndexing && cooldownExpired) {
            waitingToSpindex = true;
            ballGripTimer.resetTimer();
        }
        
        prevSlot0HasBall = slot0HasBall;
        
        // after grip delay, start spindexing
        if (waitingToSpindex && ballGripTimer.getElapsedTimeSeconds() >= BALL_GRIP_DELAY_S) {
            spindexer.stepForward();
            autoIndexing = true;
            waitingToSpindex = false;
            autoIndexTimer.resetTimer();
        }
        
        // handle auto-indexing timer - brief delay after spindexing before lowering
        if (autoIndexing) {
            if (autoIndexTimer.getElapsedTimeSeconds() < TRANSFER_LOWER_DELAY_S) {
                // keep transfer raised briefly after spindex
                transfer.raiseLever();
                transfer.runTransfer(Transfer.CrState.REVERSE);
            } else {
                // lower transfer
                autoIndexing = false;
                transfer.lowerLever();
                transfer.runTransfer(Transfer.CrState.OFF);
            }
        }
        
        // during waiting period, raise transfer to grip the ball
        if (waitingToSpindex) {
            transfer.raiseLever();
            transfer.runTransfer(Transfer.CrState.REVERSE);
        }
        
        if (!autoIndexing && !waitingToSpindex) {
            // only raise transfer if spindexer is at slot 0 and it has a ball
            if (spindexer.getCurrentSlot() == 0 && slots.hasAnyBall(0)) {
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
