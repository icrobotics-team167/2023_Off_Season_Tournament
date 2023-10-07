package frc.robot.routines;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.Queue;

import frc.robot.routines.auto.AutoState;

/**
 * An auto action to run other actions.
 */
public class Routine extends Action {

    private Queue<Action> actions;
    private Action currentAction;

    /**
     * Constructs a new auto routine.
     * 
     * @param actions The actions to run in the routine.
     */
    public Routine(Action... actions) {
        this.actions = new LinkedList<>(Arrays.asList(actions));
        setState(AutoState.INIT);
    }

    /**
     * Gets the first action in the list.
     */
    @Override
    public void init() {
        currentAction = actions.poll();
    }

    /**
     * Runs each action in the routine.
     */
    @Override
    public void periodic() {
        if (currentAction != null) {
            if (currentAction.getState() == AutoState.FINISHED) {
                currentAction = actions.poll();
                if (currentAction == null) {
                    return;
                }
            }
            currentAction.exec();
        }
    }

    /**
     * Checks if it has gone through every action in the routine.
     */
    @Override
    public boolean isDone() {
        return currentAction == null;
    }

    @Override
    public void done() {
    }

}
