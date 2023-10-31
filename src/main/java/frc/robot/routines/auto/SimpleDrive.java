package frc.robot.routines.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.routines.Action;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.turret.*;
import frc.robot.util.PeriodicTimer;

public class SimpleDrive extends Action {

    ChassisSpeeds speeds;
    double seconds;
    PeriodicTimer timer;
    private TurretPosition targetState = null;
    private boolean turretDone = true;
    private boolean runIntake = false;

    public SimpleDrive(ChassisSpeeds speeds, double seconds) {
        this.speeds = speeds;
        this.seconds = seconds;
        timer = new PeriodicTimer();
    }

    public SimpleDrive withTurret(TurretPosition position) {
        targetState = position;
        turretDone = false;
        return this;
    }

    public SimpleDrive withIntake() {
        this.runIntake = true;
        return this;
    }

    @Override
    public void init() {
        timer.reset();
    }

    @Override
    public void periodic() {
        Subsystems.driveBase.fieldOrientedDrive(speeds, Subsystems.gyro.getYaw());
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
        Subsystems.driveBase.fieldOrientedDrive(speeds, Subsystems.gyro.getYaw());
    }

    @Override
    public boolean isDone() {
        return timer.get() >= seconds;
    }

    @Override
    public void done() {
        Subsystems.driveBase.stop();
    }
}
