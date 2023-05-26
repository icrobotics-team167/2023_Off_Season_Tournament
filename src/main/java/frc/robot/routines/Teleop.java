package frc.robot.routines;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config;
import frc.robot.controls.controlschemes.ControlScheme;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.SwerveDriveBase;

public class Teleop {
    private ControlScheme controls;
    private SwerveDriveBase driveBase;

    private static final double MAX_MOVE_SPEED = Config.Settings.SwerveDrive.MAX_MOVE_SPEED;
    private static final double MAX_TURN_SPEED = Config.Settings.SwerveDrive.MAX_TURN_SPEED;
    private static final double SLOWMODE_MULT = Config.Settings.SwerveDrive.SLOWMODE_MULT;

    private LimeLight limeLight;
    // private AHRS navx = Subsystems.navx;

    public Teleop(ControlScheme controls) {
        this.controls = controls;
        driveBase = Subsystems.driveBase;
    }

    public void init() {
        driveBase.resetPosition();
    }

    public void periodic() {

        if (controls.toggleLimelight()) {
            limeLight.toggleMode();
        }

        double moveSpeed = controls.doSlowMode() ? MAX_MOVE_SPEED * SLOWMODE_MULT : MAX_MOVE_SPEED;
        double turnSpeed = controls.doSlowMode() ? MAX_TURN_SPEED * SLOWMODE_MULT : MAX_TURN_SPEED;
        Subsystems.driveBase.drive(controls.getSwerveX() * moveSpeed, controls.getSwerveY() * moveSpeed,
                controls.getSwerveTurn() * turnSpeed);
        
        // PUT DEBUG STATEMENTS BELOW THIS LINE
    }

}
