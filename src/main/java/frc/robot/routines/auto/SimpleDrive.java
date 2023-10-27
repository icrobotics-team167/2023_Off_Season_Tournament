package frc.robot.routines.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.routines.Action;
import frc.robot.subsystems.Subsystems;
import frc.robot.util.PeriodicTimer;

public class SimpleDrive extends Action {

    ChassisSpeeds speeds;
    double seconds;
    PeriodicTimer timer;

    public SimpleDrive(ChassisSpeeds speeds, double seconds) {
        this.speeds = speeds;
        this.seconds = seconds;
        timer = new PeriodicTimer();
    }

    @Override
    public void init() {
        timer.reset();
    }

    @Override
    public void periodic() {
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
