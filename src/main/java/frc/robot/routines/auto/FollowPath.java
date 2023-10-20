package frc.robot.routines.auto;

import frc.robot.Config;
import frc.robot.routines.Action;
import frc.robot.subsystems.Subsystems;
import frc.robot.util.PeriodicTimer;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class FollowPath extends Action {

    private PathPlannerTrajectory path;
    private PeriodicTimer timer;

    // TODO: Tune PIDs, this is definitely not gonna work
    private PIDController xController;
    private final double xP = 0;
    private final double xI = 0;
    private final double xD = 0;

    private PIDController yController;
    private final double yP = 0;
    private final double yI = 0;
    private final double yD = 0;

    private ProfiledPIDController rotController;
    private final double rotP = 0;
    private final double rotI = 0;
    private final double rotD = 0;

    private HolonomicDriveController driveController;

    public FollowPath(String path) {
        super();
    }

    @Override
    public void init() {

    }

    @Override
    public void periodic() {

    }

    @Override
    public boolean isDone() {
        return false;
    }

    @Override
    public void done() {
    }

}
