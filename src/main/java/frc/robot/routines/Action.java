package frc.robot.routines;

import frc.robot.routines.auto.AutoState;

/**
 * An abstraction for autonomous actions.
 */
public abstract class Action {

    public abstract void init();

    public abstract void periodic();

    public abstract boolean isDone();

    public abstract void done();

    protected AutoState state;

    public Action() {
        state = AutoState.INIT;
    }

    /**
     * Runs every tick of auto.
     */
    public void exec() {
        // If something goes horribly wrong and there's nothing loaded into state, do nothing
        if (state == null) {
            return;
        }

        // Switch statement to choose actions depending on current state.
        // See AutoState.java for an explanation on what each state does.
        switch (state) {
            case INIT:
                init();
                setState(AutoState.PERIODIC);
                break;
            case PERIODIC:
                periodic();
                if (isDone()) {
                    setState(AutoState.DONE);
                }
                break;
            case DONE:
                done();
                setState(AutoState.FINISHED);
                break;
            case FINISHED:
                break;
            default:
                break;
        }
    }

    public void setState(AutoState state) {
        this.state = state;
    }

    public AutoState getState() {
        return state;
    }

}
