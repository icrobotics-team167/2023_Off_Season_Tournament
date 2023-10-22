package frc.robot.routines.auto;

import frc.robot.Config;
import frc.robot.routines.Action;
import frc.robot.subsystems.Subsystems;
import frc.robot.util.PeriodicTimer;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.pathplanner.lib.util.PPLibTelemetry;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class FollowPath extends Action {

    private String pathName;
    private PathPlannerPath pathFile;
    private PathPlannerTrajectory trajectory;
    private PeriodicTimer timer;

    // TODO: Tune PIDs, this is definitely not gonna work
    private PIDController xController;
    private final double xP = 2;
    private final double xI = 0;
    private final double xD = 0;

    private PIDController yController;
    private final double yP = 2;
    private final double yI = 0;
    private final double yD = 0;

    private ProfiledPIDController rotController;
    private final double rotP = -2;
    private final double rotI = 0;
    private final double rotD = 0;

    private HolonomicDriveController driveController;

    public FollowPath(String path) {
        super();
        this.pathName = path;
        pathFile = PathPlannerPath.fromPathFile(path);
        // Debug
        if (pathFile == null) {
            DriverStation.reportWarning("FollowPath: " + pathName + ".path failed to load!", false);
        } else {
            System.out.println("FollowPath: Loaded " + pathName + ".path");
        }
        PPLibTelemetry.registerHotReloadPath(pathName, pathFile);
        this.trajectory = new PathPlannerTrajectory(pathFile, new ChassisSpeeds());
        this.xController = new PIDController(xP, xI, xD);
        this.yController = new PIDController(yP, yI, yD);
        this.rotController = new ProfiledPIDController(rotP, rotI, rotD, new Constraints(
                Config.Settings.SwerveDrive.MAX_TURN_SPEED, Config.Settings.SwerveDrive.MAX_TURN_ACCEL));
        this.driveController = new HolonomicDriveController(xController, yController, rotController);
        this.timer = new PeriodicTimer();
    }

    @Override
    public void init() {
        Pose2d initPos = new Pose2d(trajectory.sample(0).positionMeters, trajectory.sample(0).targetHolonomicRotation);
        Subsystems.driveBase.setPose(initPos);
        SmartDashboard.putString("FollowPath.path", pathName);
        timer.reset();
        PPLibTelemetry.setCurrentPath(pathFile);
        PPLibTelemetry.setCurrentPose(initPos);
    }

    @Override
    public void periodic() {
        State state = trajectory.sample(timer.get());
        ChassisSpeeds driveCommands = driveController.calculate(
                Subsystems.driveBase.getPose(), // Current Pose
                state.getTargetHolonomicPose(), // Target Pose
                state.velocityMps, // Drive velocity
                state.targetHolonomicRotation); // Target rotation

        ChassisSpeeds robotVelocities = Subsystems.driveBase.getVelocity();
        double velocityError = new Translation2d(robotVelocities.vxMetersPerSecond, robotVelocities.vyMetersPerSecond)
                .getNorm()
                / new Translation2d(driveCommands.vxMetersPerSecond, driveCommands.vyMetersPerSecond).getNorm();
        // Telemetry
        PPLibTelemetry.setVelocities(
                new Translation2d(robotVelocities.vxMetersPerSecond, robotVelocities.vyMetersPerSecond).getNorm(),
                new Translation2d(driveCommands.vxMetersPerSecond, driveCommands.vyMetersPerSecond).getNorm(),
                robotVelocities.omegaRadiansPerSecond,
                driveCommands.omegaRadiansPerSecond);
        PPLibTelemetry
                .setPathInaccuracy(state.positionMeters.getDistance(Subsystems.driveBase.getPose().getTranslation()));
        PPLibTelemetry.setCurrentPose(Subsystems.driveBase.getPose());
        SmartDashboard.putNumber("FollowPath.xPIDOut", driveCommands.vxMetersPerSecond);
        SmartDashboard.putNumber("FollowPath.yPIDOut", driveCommands.vyMetersPerSecond);
        SmartDashboard.putNumber("FollowPath.velocityError", velocityError);

        // driveCommands.vxMetersPerSecond /= velocityError;
        // driveCommands.vyMetersPerSecond /= velocityError;
        Subsystems.driveBase.drive(driveCommands);
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
