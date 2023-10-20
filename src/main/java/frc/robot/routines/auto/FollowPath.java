package frc.robot.routines.auto;

import frc.robot.Config;
import frc.robot.routines.Action;
import frc.robot.subsystems.Subsystems;
import frc.robot.util.PeriodicTimer;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

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
        this.path = PathPlanner.loadPath(path, Config.Settings.SwerveDrive.MAX_MOVE_SPEED, Config.Settings.SwerveDrive.MAX_MOVE_ACCEL); 
        xController = new PIDController(xP, xI, xD);
        yController = new PIDController(yP, yI, yD);
        rotController = new ProfiledPIDController(rotP, rotI, rotD,
                new TrapezoidProfile.Constraints(Config.Settings.SwerveDrive.MAX_TURN_SPEED, Config.Settings.SwerveDrive.MAX_TURN_ACCEL));
        driveController = new HolonomicDriveController(xController, yController, rotController);
        timer = new PeriodicTimer();
    }

    @Override
    public void init() {
        Subsystems.driveBase.setPose(path.getInitialPose());
        timer.reset();
        SmartDashboard.putNumber("StartXPos", Subsystems.driveBase.getPose().getX());
        SmartDashboard.putNumber("StartYPos", Subsystems.driveBase.getPose().getY());
    }

    @Override
    public void periodic() {
        PathPlannerState state = (PathPlannerState) path.sample(timer.get());
        Subsystems.driveBase.fieldOrientedDrive(
                driveController.calculate(Subsystems.driveBase.getPose(), state, state.poseMeters.getRotation()),
                Subsystems.gyro.getYaw());
        SmartDashboard.putNumber("xPosWant", state.poseMeters.getX());
        SmartDashboard.putNumber("yPosWant", state.poseMeters.getY());

    }

    @Override
    public boolean isDone() {
        // return timer.get() >= path.getTotalTimeSeconds();
        return false;
    }

    @Override
    public void done() {
        Subsystems.driveBase.stop();
    }

}
