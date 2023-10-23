package frc.robot.routines.auto;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.pathplanner.lib.util.PPLibTelemetry;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config;
import frc.robot.routines.Action;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.turret.TurretPosition;
import frc.robot.util.PeriodicTimer;

public class DriveStraight extends Action {

    private Pose2d targetPose;
    private PathPlannerPath path;
    private PathPlannerTrajectory trajectory;
    private PeriodicTimer timer;

    private boolean firstMove;

    private PIDController xController;
    private final double xP = Config.Settings.SwerveDrive.AutoPIDs.xP;
    private final double xI = Config.Settings.SwerveDrive.AutoPIDs.xI;
    private final double xD = Config.Settings.SwerveDrive.AutoPIDs.xD;

    private PIDController yController;
    private final double yP = Config.Settings.SwerveDrive.AutoPIDs.yP;
    private final double yI = Config.Settings.SwerveDrive.AutoPIDs.yI;
    private final double yD = Config.Settings.SwerveDrive.AutoPIDs.yD;

    private ProfiledPIDController rotController;
    private final double rotP = Config.Settings.SwerveDrive.AutoPIDs.rotP;
    private final double rotI = Config.Settings.SwerveDrive.AutoPIDs.rotI;
    private final double rotD = Config.Settings.SwerveDrive.AutoPIDs.rotD;

    private HolonomicDriveController driveController;

    private TurretPosition targetState = null;
    private boolean turretDone = true;

    private boolean runIntake = false;

    public DriveStraight(Pose2d targetPose) {
        this(targetPose, false);
    }

    public DriveStraight(Pose2d targetPose, boolean firstMove) {
        super();
        this.targetPose = targetPose;
        xController = new PIDController(xP, xI, xD);
        yController = new PIDController(yP, yI, yD);
        rotController = new ProfiledPIDController(rotP, rotI, rotD, new Constraints(
                Config.Settings.SwerveDrive.MAX_TURN_SPEED, Config.Settings.SwerveDrive.MAX_TURN_ACCEL));
        this.driveController = new HolonomicDriveController(xController, yController, rotController);
        this.firstMove = firstMove;
        this.timer = new PeriodicTimer();
    }

    @Override
    public void init() {
        if (firstMove) {
            Pose2d initPos = trajectory.getInitialTargetHolonomicPose();
            Subsystems.driveBase.setPose(initPos);
            PPLibTelemetry.setCurrentPose(initPos);
        }

        Translation2d travelVector = targetPose.getTranslation().minus(Subsystems.driveBase.getPose().getTranslation());
        Rotation2d travelDir = travelVector.getAngle();

        path = new PathPlannerPath(
                PathPlannerPath.bezierFromPoses(
                        new Pose2d(Subsystems.driveBase.getPose().getTranslation(), travelDir),
                        new Pose2d(targetPose.getTranslation(), travelDir.rotateBy(Rotation2d.fromDegrees(180)))),
                new PathConstraints(
                        Config.Settings.SwerveDrive.MAX_MOVE_SPEED,
                        Config.Settings.SwerveDrive.MAX_MOVE_ACCEL,
                        Config.Settings.SwerveDrive.MAX_TURN_SPEED,
                        Config.Settings.SwerveDrive.MAX_TURN_ACCEL),
                new GoalEndState(0, targetPose.getRotation()));
        trajectory = new PathPlannerTrajectory(path, new ChassisSpeeds());

        SmartDashboard.putString("FollowPath.path", "DriveStraight");
        PPLibTelemetry.setCurrentPath(path);

        timer.reset();
    }

    public DriveStraight withIntake() {
        this.runIntake = true;
        return this;
    }

    public DriveStraight withTurret(TurretPosition targetState) {
        this.targetState = targetState;
        this.turretDone = false;
        return this;
    }

    @Override
    public void periodic() {
        State state = trajectory.sample(timer.get() / 2);
        ChassisSpeeds driveCommands = driveController.calculate(
                Subsystems.driveBase.getPose(), // Current Pose
                new Pose2d(state.getTargetHolonomicPose().getX(), state.getTargetHolonomicPose().getY(), state.heading), // Target
                                                                                                                         // Pose
                state.velocityMps, // Drive velocity
                state.targetHolonomicRotation);

        ChassisSpeeds robotVelocities = Subsystems.driveBase.getVelocity();
        double velocityError = new Translation2d(robotVelocities.vxMetersPerSecond, robotVelocities.vyMetersPerSecond)
                .getNorm()
                / new Translation2d(driveCommands.vxMetersPerSecond, driveCommands.vyMetersPerSecond).getNorm();
        // Telemetry
        PPLibTelemetry.setVelocities(
                new Translation2d(robotVelocities.vxMetersPerSecond, robotVelocities.vyMetersPerSecond).getNorm(),
                new Translation2d(driveCommands.vxMetersPerSecond, driveCommands.vyMetersPerSecond).getNorm(),
                robotVelocities.omegaRadiansPerSecond,
                -driveCommands.omegaRadiansPerSecond);
        PPLibTelemetry
                .setPathInaccuracy(state.positionMeters.getDistance(Subsystems.driveBase.getPose().getTranslation()));
        PPLibTelemetry.setCurrentPose(Subsystems.driveBase.getPose());
        PPLibTelemetry.setTargetPose(state.getTargetHolonomicPose());
        SmartDashboard.putNumber("FollowPath.xPIDOut", driveCommands.vxMetersPerSecond);
        SmartDashboard.putNumber("FollowPath.yPIDOut", driveCommands.vyMetersPerSecond);
        SmartDashboard.putNumber("FollowPath.rotPIDOut", driveCommands.omegaRadiansPerSecond);
        SmartDashboard.putNumber("FollowPath.velocityError", velocityError);
        SmartDashboard.putNumber("FollowPath.rotVelError",
                driveCommands.omegaRadiansPerSecond - robotVelocities.omegaRadiansPerSecond);

        // driveCommands.vxMetersPerSecond /= velocityError;
        // driveCommands.vyMetersPerSecond /= velocityError;
        Subsystems.driveBase.drive(driveCommands);

        if (targetState != null && !turretDone) {
            turretDone = Subsystems.turret.moveTo(targetState);
        } else {
            Subsystems.turret.stop();
        }

        if (runIntake) {
            Subsystems.claw.intake();
        } else {
            Subsystems.claw.stop();
        }
    }

    @Override
    public boolean isDone() {
        return timer.get() / 2 >= trajectory.getTotalTimeSeconds() && turretDone;
    }

    @Override
    public void done() {
        Subsystems.driveBase.stop();
        Subsystems.turret.stop();
        Subsystems.claw.stop();
    }

}
