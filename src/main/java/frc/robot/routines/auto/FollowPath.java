package frc.robot.routines.auto;

import frc.robot.Config;
import frc.robot.routines.Action;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.turret.TurretPosition;
import frc.robot.util.PeriodicTimer;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

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

    private TurretPosition targetState = null;
    private boolean turretDone = true;

    private boolean runIntake = false;

    public FollowPath(PathPlannerTrajectory path, double timeout) {
        super();
        this.path = path;
        this.timeout = timeout;
        xController = new PIDController(xP, xI, xD);
        yController = new PIDController(yP, yI, yD);
        rotController = new ProfiledPIDController(rotP, rotI, rotD, 
                new TrapezoidProfile.Constraints(Config.Settings.SwerveDrive.MAX_TURN_SPEED, 0));
        driveController = new HolonomicDriveController(xController, yController, null);
        timer = new PeriodicTimer();
    }

    public FollowPath withIntake() {
        runIntake = true;
        return this;
    }

    public FollowPath withTurret(TurretPosition target) {
        targetState = target;
        turretDone = false;
        return this;
    }

    @Override
    public void init() {
        timer.reset();
    }

    @Override
    public void periodic() {
        PathPlannerState state = (PathPlannerState) path.sample(timer.get());
        if(targetState != null && !turretDone) {
            turretDone = Subsystems.turret.moveTo(targetState);
        } else {
            Subsystems.turret.stop();
        }

        if(runIntake) {
            Subsystems.claw.intake();
        } else {
            Subsystems.turret.stop();
        }
    }

    @Override
    public boolean isDone() {
        return timer.get() >= timeout && turretDone;
    }

    @Override
    public void done() {
        Subsystems.driveBase.stop();
        Subsystems.turret.stop();
        Subsystems.claw.stop();
    }

}
