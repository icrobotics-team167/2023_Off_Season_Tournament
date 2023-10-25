package frc.robot.routines.auto;

import frc.robot.Config;
import frc.robot.routines.Action;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.turret.TurretPosition;
import frc.robot.util.MathUtils;
import frc.robot.util.PeriodicTimer;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.pathplanner.lib.util.PPLibTelemetry;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class FollowPath extends Action {

    private String pathName;
    private PathPlannerPath pathFile;
    private PathPlannerTrajectory trajectory;
    private PeriodicTimer timer;

    private Pose2d startPos;

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

    public FollowPath(String path, Pose2d startPos) {
        super();
        this.pathName = path;
        pathFile = PathPlannerPath.fromPathFile(path);
        // Debug
        if (pathFile == null) {
            DriverStation.reportWarning("FollowPath: " + pathName + ".path failed to load!", false);
        } else {
            System.out.println("FollowPath: Loaded " + pathName + ".path");
        }
        this.trajectory = new PathPlannerTrajectory(pathFile, new ChassisSpeeds());
        
        this.xController = new PIDController(xP, xI, xD);
        this.yController = new PIDController(yP, yI, yD);
        this.rotController = new ProfiledPIDController(rotP, rotI, rotD, new Constraints(
                Config.Settings.SwerveDrive.MAX_TURN_SPEED, Config.Settings.SwerveDrive.MAX_TURN_ACCEL));
        this.driveController = new HolonomicDriveController(xController, yController, rotController);
        this.startPos = startPos;
        this.timer = new PeriodicTimer();
    }

    public FollowPath(String path) {
        this(path, null);
    }

    public FollowPath withIntake() {
        this.runIntake = true;
        return this;
    }

    public FollowPath withTurret(TurretPosition targetState) {
        this.targetState = targetState;
        this.turretDone = false;
        return this;
    }

    @Override
    public void init() {
        if (startPos != null) {
            startPos = MathUtils.flipPos(startPos);
            Subsystems.driveBase.setPose(startPos);
            PPLibTelemetry.setCurrentPose(startPos);
        } else {
            PPLibTelemetry.setCurrentPose(Subsystems.driveBase.getPose());
        }

        SmartDashboard.putString("FollowPath.path", pathName);
        PPLibTelemetry.setCurrentPath(pathFile);

        timer.reset();
    }

    @Override
    public void periodic() {
        // State state = trajectory.sample(timer.get());
        State state = mirrorState(trajectory.sample(timer.get() / 2));

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

        Subsystems.driveBase.drive(driveCommands);

        if (targetState != null && !turretDone) {
            turretDone = Subsystems.turret.moveTo(targetState);
        } else {
            Subsystems.turret.stop();
        }

        if (runIntake) {
            Subsystems.claw.intake(0.5);
        } else {
            Subsystems.claw.stop();
        }

    }

    @Override
    public boolean isDone() {
        // return timer.get() >= trajectory.getTotalTimeSeconds() && turretDone;
        return timer.get() / 2 >= trajectory.getTotalTimeSeconds() && turretDone;
    }

    @Override
    public void done() {
        Subsystems.driveBase.stop();
        Subsystems.turret.stop();
        Subsystems.claw.stop();
    }

    /**
     * <p>
     * Flips the X axis and rotation of a state. Does nothing if
     * Config.Settings.ASYMMETRICAL_FIELD = false or if the robot is on the Blue
     * Alliance.
     * <p>
     * On fields that aren't symmetrical down the X axis, modifying positions to
     * handle asymmetrical X axes is neccesary when on the Red Alliance.
     * 
     * @param state Original state
     * @return New state
     */
    private State mirrorState(State state) {
        if (!Config.Settings.ASYMMETRICAL_FIELD || DriverStation.getAlliance() == Alliance.Blue) {
            return state;
        }
        State mirroredState = new State();
        mirroredState.accelerationMpsSq = state.accelerationMpsSq;
        mirroredState.constraints = state.constraints;
        mirroredState.curvatureRadPerMeter = state.curvatureRadPerMeter;
        mirroredState.headingAngularVelocityRps = state.headingAngularVelocityRps;
        mirroredState.positionMeters = new Translation2d(Config.Settings.FIELD_WIDTH - state.positionMeters.getX(),
                state.positionMeters.getY());
        mirroredState.targetHolonomicRotation = state.targetHolonomicRotation.unaryMinus()
                .plus(Rotation2d.fromDegrees(180));
        mirroredState.timeSeconds = state.timeSeconds;
        mirroredState.velocityMps = state.velocityMps;
        return mirroredState;
    }

}
