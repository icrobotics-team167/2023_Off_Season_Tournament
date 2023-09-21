package frc.robot.routines.auto;

import frc.robot.routines.Action;

/**
 * A null action, as a fallback.
 */
public class NullAction extends Action {

    @Override
    public void init() {
    }

    @Override
    public void periodic() {

    }

    @Override
    public boolean isDone() {
        return true;
    }

    @Override
    public void done() {
    }

}
