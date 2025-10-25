package org.firstinspires.ftc.teamcode.auto.state;

import org.firstinspires.ftc.teamcode.auto.deposit.DepositController;
import org.firstinspires.ftc.teamcode.auto.phases.MovePhaseController;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

public class AutoStateMachine {
    private AutoState state = AutoState.MOVE;

    private final MovePhaseController move;
    private final DepositController deposit;
    private final TelemetryHelper tele;

    public AutoStateMachine(MovePhaseController move, DepositController deposit, TelemetryHelper tele) {
        this.move = move;
        this.deposit = deposit;
        this.tele = tele;
    }

    public AutoState getState() {
        return state;
    }

    public void update() {
        switch (state) {
            case MOVE:
                MovePhaseController.Result r = move.update(null);
                break;
            case DEPOSIT:
                boolean done = deposit.update();
                if (done) state = AutoState.DONE;
                break;
            case DONE:
                break;
        }
        tele.addLine("--- STATE ---").addData("State", state::name);
    }
}
