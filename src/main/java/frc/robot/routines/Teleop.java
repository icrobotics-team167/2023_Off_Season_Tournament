package frc.robot.routines;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config;
import frc.robot.controls.controlschemes.ControlScheme;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.SwerveDriveBase;
import frc.robot.subsystems.turret.*;

public class Teleop {
    private ControlScheme controls;
    private SwerveDriveBase driveBase;

    private static final double MAX_MOVE_SPEED = Config.Settings.SwerveDrive.TELEOP_MAX_MOVE_SPEED;
    private static final double MAX_TURN_SPEED = Config.Settings.SwerveDrive.TELEOP_MAX_TURN_SPEED;
    private static final double SLOWMODE_MULT = Config.Settings.SwerveDrive.SLOWMODE_MULT;

    private LimeLight limeLight = Subsystems.limeLight;
    private Turret turret = Subsystems.turret;
    private Claw claw = Subsystems.claw;

    private TurretPosition targetState = null;
    private TurretPosition holdState;

    public Teleop(ControlScheme controls) {
        this.controls = controls;
        driveBase = Subsystems.driveBase;
    }

    /**
     * Runs once at the start of teleop
     */
    public void init() {
        holdState = turret.getPosition();
    }

    /**
     * Runs every tick of teleop
     */
    public void periodic() {

        // Driving
        // If slowmode is on, multiply max move/turn speed by slowmode's speed
        // multiplier.
        // Otherwise, keep max move/turn speed as is.
        double moveSpeed = controls.doSlowMode() ? MAX_MOVE_SPEED * SLOWMODE_MULT : MAX_MOVE_SPEED;
        double turnSpeed = controls.doSlowMode() ? MAX_TURN_SPEED * SLOWMODE_MULT : MAX_TURN_SPEED;
        SmartDashboard.putNumber("Teleop.moveSpeed", moveSpeed);
        SmartDashboard.putNumber("Teleop.turnSpeed", turnSpeed);

        double controlMagnitude = Math.hypot(controls.getSwerveX(), controls.getSwerveY());
        double controlMultiplier = controlMagnitude > 1 ? 1 / controlMagnitude : 1;
        double forwardsVel = controls.getSwerveY() * moveSpeed * controlMultiplier;
        double sideVel = controls.getSwerveX() * moveSpeed * controlMultiplier;
        double turnVel = controls.getSwerveTurn() * turnSpeed;
        SmartDashboard.putNumber("Teleop.forwardsVel", forwardsVel);
        SmartDashboard.putNumber("Teleop.sideVel", sideVel);
        SmartDashboard.putNumber("Teleop.turnVel", turnVel);

        if (controls.resetYaw()) {
            Subsystems.gyro.resetYaw();
        }

        if (Config.Settings.FIELD_ORIENTED_DRIVE) {
            Subsystems.driveBase.fieldOrientedDrive(forwardsVel, // Forward/backwards city
                    sideVel, // Left/right velocity
                    turnVel, // Turn velocity
                    Subsystems.gyro.getYaw()); // Current orientation
        } else {
            Subsystems.driveBase.drive(forwardsVel,
                    sideVel,
                    turnVel);
        }

        if (controls.toggleLimelight()) {
            limeLight.toggleMode();
        }
        // turret.setSlowMode(controls.doSlowTurret());

        // Code so that u only have to press one button and it will automatically go to
        // it.

        if (controls.doResetTurret()) {
            targetState = TurretPosition.INITIAL;
        } else if (controls.doAutoPickup()) {
            targetState = TurretPosition.INTAKE;
        } else if (controls.doPlayerStation()) {
            targetState = TurretPosition.PLAYER_STATION;
        } else if (controls.doConeHigh()) {
            targetState = TurretPosition.CONE_HIGH;
        } else if (controls.doConeMid()) {
            targetState = TurretPosition.CONE_MID;
        } else if (controls.doCubeMid()) {
            targetState = TurretPosition.CUBE_MID;
        } else if (controls.doCubeHigh()) {
            targetState = TurretPosition.CUBE_HIGH;
        }

        if (Math.abs(controls.getArmPivot()) > 0 || Math.abs(controls.getArmExtend()) > 0) {
            targetState = null;
            turret.move(controls.getArmPivot(), controls.getArmExtend());
            holdState = turret.getPosition();
        } else {
            if (targetState != null) {
                turret.moveTo(targetState);
            } else {
                turret.moveTo(holdState);
            }
        }

        if (controls.intake()) {
            claw.intake();
        } else if (controls.outtake()) {
            claw.outtake();
        } else {
            claw.stop();
        }

        // PUT DEBUG STATEMENTS HERE
        SmartDashboard.putNumber("Pivot.position", Subsystems.turret.getPosition().pivotAngle());
        SmartDashboard.putNumber("ExtendRetract.position", Subsystems.turret.getPosition().extensionPosition());
    }

}
