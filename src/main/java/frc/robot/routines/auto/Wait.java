package frc.robot.routines.auto;

import frc.robot.routines.Action;
import frc.robot.subsystems.Subsystems;
import frc.robot.util.PeriodicTimer;

/**
 * An auto action that does waits for a specified period of time.
 */
public class Wait extends Action {

    private PeriodicTimer timer;
    private double seconds;

    /**
     * Constructs an action to wait for a specified period of time.
     * 
     * @param seconds The time to wait for, in seconds.
     */
    public Wait(double seconds) {
        super();
        timer = new PeriodicTimer();
        this.seconds = seconds;
    }

    @Override
    public void init() {
        timer.reset();
    }

    @Override
    public void periodic() {
        Subsystems.driveBase.stop();
    }

    @Override
    public boolean isDone() {
        return timer.hasElapsed(seconds);
    }

    @Override
    public void done() {

    }

}
