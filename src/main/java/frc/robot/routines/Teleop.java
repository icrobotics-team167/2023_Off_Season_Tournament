package frc.robot.routines;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
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

    // private LimeLight limeLight;
    private AHRS navx = Subsystems.navx;

    public Teleop(ControlScheme controls) {
        this.controls = controls;
        driveBase = Subsystems.driveBase;
    }

    /**
     * Runs once at the start of teleop
     */
    public void init() {
        driveBase.resetPosition();
    }

    /**
     * Runs every tick of teleop
     */
    public void periodic() {

        // Driving
        // If slowmode is on, multiply max move/turn speed by slowmode's speed multiplier.
        // Otherwise, keep max move/turn speed as is.
        double moveSpeed = controls.doSlowMode() ? MAX_MOVE_SPEED * SLOWMODE_MULT : MAX_MOVE_SPEED;
        double turnSpeed = controls.doSlowMode() ? MAX_TURN_SPEED * SLOWMODE_MULT : MAX_TURN_SPEED;
        Subsystems.driveBase.fieldOrientedDrive(controls.getSwerveY() * moveSpeed, // Forward/backwards velocity
                controls.getSwerveX() * moveSpeed, // Left/right velocity
                controls.getSwerveTurn() * turnSpeed, // Turn velocity
                Rotation2d.fromDegrees(-navx.getYaw())); // Current orientation

        // PUT DEBUG STATEMENTS BELOW THIS LINE
        SmartDashboard.putNumber("Teleop.navxYaw", -navx.getYaw());
    }

}
