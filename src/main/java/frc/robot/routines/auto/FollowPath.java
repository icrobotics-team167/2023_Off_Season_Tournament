package frc.robot.routines.auto;

import frc.robot.Config;
import frc.robot.routines.Action;
import frc.robot.subsystems.Subsystems;
import frc.robot.util.PeriodicTimer;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FollowPath extends Action {

    private PathPlannerTrajectory path;
    private double timeout;
    private PeriodicTimer timer;

    // TODO: Tune PIDs, this is definitely not gonna work
    private PIDController xController;
    private final double xP = 1;
    private final double xI = 0;
    private final double xD = 0.5;

    private PIDController yController;
    private final double yP = 1;
    private final double yI = 0;
    private final double yD = 0.5;

    private ProfiledPIDController rotController;
    private final double rotP = 1;
    private final double rotI = 0;
    private final double rotD = 0.5;

    private HolonomicDriveController driveController;

    public FollowPath(String path, double timeout) {
        super();
        this.path = PathPlanner.loadPath(path, 4, 1); // Temp values
        this.timeout = timeout;
        xController = new PIDController(xP, xI, xD);
        yController = new PIDController(yP, yI, yD);
        rotController = new ProfiledPIDController(rotP, rotI, rotD,
                new TrapezoidProfile.Constraints(Config.Settings.SwerveDrive.MAX_TURN_SPEED, 0));
        driveController = new HolonomicDriveController(xController, yController, rotController);
        timer = new PeriodicTimer();
    }

    @Override
    public void init() {
        timer.reset();
    }

    @Override
    public void periodic() {
        PathPlannerState state = (PathPlannerState) path.sample(timer.get());
        Rotation2d heading = null; // TODO: Figure out the math for this
        Subsystems.driveBase.drive(driveController.calculate(Subsystems.driveBase.getPose(), state, state.poseMeters.getRotation()));

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
