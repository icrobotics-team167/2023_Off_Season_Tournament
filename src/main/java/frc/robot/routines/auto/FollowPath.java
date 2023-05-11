package frc.robot.routines.auto;

import frc.robot.routines.Action;
import frc.robot.subsystems.Subsystems;
import frc.robot.util.PeriodicTimer;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class FollowPath extends Action {

    private PathPlannerTrajectory path;
    private double timeout;
    private PeriodicTimer timer;

    public FollowPath(PathPlannerTrajectory path, double timeout) {
        super();
        this.path = path;
        this.timeout = timeout;
        timer = new PeriodicTimer();
    }

    @Override
    public void init() {
        timer.reset();
    }

    @Override
    public void periodic() {
        PathPlannerState state = (PathPlannerState)path.sample(timer.get());
        Subsystems.driveBase.drive(new ChassisSpeeds(timeout, timeout, timeout));
    }

    @Override
    public boolean isDone() {
        return timer.get() >= timeout;
    }

    @Override
    public void done() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'done'");
    }
    
}
