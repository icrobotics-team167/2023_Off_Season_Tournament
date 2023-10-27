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


    public SimpleDrive(ChassisSpeeds speeds, double seconds) {
        this.speeds = speeds;
        this.seconds = seconds;
        timer = new PeriodicTimer();
    }

    @Override
    public void init() {
        timer.reset();
        TurretMove(TurretPosition.CONE_HIGH);
    }

    @Override
    public void periodic() {
        Subsystems.driveBase.fieldOrientedDrive(speeds, Subsystems.gyro.getYaw());
        if (targetState != null && !turretDone) {
            turretDone = Subsystems.turret.moveTo(targetState);
        } else {
            Subsystems.turret.stop();
        }
        new ChassisSpeeds driveCommands = null;
        Subsystems.driveBase.drive(driveCommands);
    }

    @Override
    public boolean isDone() {
        return timer.get() >= seconds;
    }

    @Override
    public void done() {
        Subsystems.driveBase.stop();
    }

    // //public FollowPath withTurret(TurretPosition targetState) {
    //     thitargetStates. = targetState;
    //     this.turretDone = false;
    //     return this;
    // }
    @Override
    public void turretMove(TurretPosition target){
        this.targetState = target;
        this.turretDone = false;
    }
}
