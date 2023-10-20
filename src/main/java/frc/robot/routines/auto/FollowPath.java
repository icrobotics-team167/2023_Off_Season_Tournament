package frc.robot.routines.auto;

import frc.robot.Config;
import frc.robot.routines.Action;
import frc.robot.subsystems.Subsystems;
import frc.robot.util.PeriodicTimer;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;

public class FollowPath extends Action {

    private String pathName;
    private PathPlannerTrajectory trajectory;
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
        this.pathName = path;
        this.trajectory = new PathPlannerTrajectory(PathPlannerPath.fromPathFile(path), null);
        this.xController = new PIDController(xP, xI, xD);
        this.yController = new PIDController(yP, yI, yD);
        this.rotController = new ProfiledPIDController(rotP, rotI, rotD, new Constraints(
                Config.Settings.SwerveDrive.MAX_TURN_SPEED, Config.Settings.SwerveDrive.MAX_TURN_ACCEL));
        this.driveController = new HolonomicDriveController(xController, yController, rotController);
        this.timer = new PeriodicTimer();
    }

    @Override
    public void init() {
        Translation2d initPos = trajectory.sample(0).positionMeters;
        SmartDashboard.putString("FollowPath.path", pathName);
        SmartDashboard.putNumber("FollowPath.PathStartX", initPos.getX());
        SmartDashboard.putNumber("FollowPath.PathStartY", initPos.getY());
        timer.reset();
    }

    @Override
    public void periodic() {
        State state = trajectory.sample(timer.get());
        Subsystems.driveBase.fieldOrientedDrive(
                driveController.calculate(
                        Subsystems.driveBase.getPose(), // Current Pose
                        state.getTargetHolonomicPose(), // Target Pose
                        state.velocityMps, // Drive velocity
                        state.targetHolonomicRotation), // Target rotation
                Subsystems.gyro.getYaw()); // Current rotation
        SmartDashboard.putNumber("FollowPath.targetX", state.positionMeters.getX());
        SmartDashboard.putNumber("FollowPath.targetY", state.positionMeters.getY());
    }

    @Override
    public boolean isDone() {
        return timer.get() >= trajectory.getTotalTimeSeconds();
    }

    @Override
    public void done() {
        Subsystems.driveBase.stop();
    }

}
