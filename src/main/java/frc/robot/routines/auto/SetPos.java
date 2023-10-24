package frc.robot.routines.auto;

import com.pathplanner.lib.util.PPLibTelemetry;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.routines.Action;
import frc.robot.subsystems.Subsystems;
import frc.robot.util.MathUtils;

public class SetPos extends Action {

    private Pose2d pos;

    public SetPos(Pose2d pos) {
        this.pos = pos;
    }

    @Override
    public void init() {
        pos = MathUtils.flipPos(pos);
        Subsystems.driveBase.setPose(pos);
        PPLibTelemetry.setCurrentPose(pos);
    }

    @Override
    public void periodic() {
    }

    @Override
    public boolean isDone() {
        return true;
    }

    @Override
    public void done() {
    }
    
}
