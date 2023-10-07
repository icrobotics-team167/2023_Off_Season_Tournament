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
        // If slowmode is on, multiply max move/turn speed by slowmode's speed
        // multiplier.
        // Otherwise, keep max move/turn speed as is.
        double moveSpeed = controls.doSlowMode() ? MAX_MOVE_SPEED * SLOWMODE_MULT : MAX_MOVE_SPEED;
        double turnSpeed = controls.doSlowMode() ? MAX_TURN_SPEED * SLOWMODE_MULT : MAX_TURN_SPEED;
        SmartDashboard.putNumber("Teleop.moveSpeed", moveSpeed);
        SmartDashboard.putNumber("Teleop.turnSpeed", turnSpeed);

        double forwardsVel = controls.getSwerveY() * moveSpeed;
        double sideVel = controls.getSwerveX() * moveSpeed;
        double turnVel = controls.getSwerveTurn() * turnSpeed;
        SmartDashboard.putNumber("Teleop.forwardsVel", forwardsVel);
        SmartDashboard.putNumber("Teleop.sideVel", sideVel);
        SmartDashboard.putNumber("Teleop.turnVel", turnVel);

        Subsystems.driveBase.fieldOrientedDrive(forwardsVel, // Forward/backwards velocity
                sideVel, // Left/right velocity
                turnVel, // Turn velocity
                Rotation2d.fromDegrees(-navx.getYaw())); // Current orientation

        // PUT DEBUG STATEMENTS BELOW THIS LINE
        SmartDashboard.putNumber("Teleop.navxYaw", -navx.getYaw());

        if (controls.toggleLimelight()) {
            limeLight.toggleMode();
        }
        // turret.setSlowMode(controls.doSlowTurret());

        // Code so that u only have to press one button and it will automatically go to
        // it.

        int swivelOffset = controls.getPositionOffset();
        if (controls.doResetTurret()) {
            targetState = TurretPosition.INITIAL;
        } else if (controls.doAutoPickup()) {
            targetState = TurretPosition.INTAKE.withSwivel(turret.getPosition().swivelAngle()).addSwivelOffset(-swivelOffset);
        } else if (controls.doPlayerStation()) {
            targetState = TurretPosition.PLAYER_STATION.withSwivel(turret.getPosition().swivelAngle()).addSwivelOffset(-swivelOffset);
        } else if (controls.doAutoHigh()) {
            targetState = TurretPosition.HIGH_MID;
        } else if (controls.doAutoMid()) {
            targetState = TurretPosition.MID_MID;
        } else if (controls.doAutoHighRight()) {
            targetState = TurretPosition.HIGH_RIGHT;
        } else if (controls.doAutoMidRight()) {
            targetState = TurretPosition.MID_RIGHT;
        } else if (controls.doAutoHighLeft()) {
            targetState = TurretPosition.HIGH_LEFT;
        } else if (controls.doAutoMidLeft()) {
            targetState = TurretPosition.MID_LEFT;
        } else if (controls.offsetUpdated()) {
            targetState = TurretPosition.INITIAL;
        }
        double swivel = controls.doUnlockSwivel() ? controls.getArmSwivel() : 0;
        if (Math.abs(controls.getArmPivot()) > 0 || Math.abs(swivel) > 0 || Math.abs(controls.getArmExtend()) > 0) {
            targetState = null;
            turret.move(controls.getArmPivot(), swivel, controls.getArmExtend());
            holdState = turret.getPosition();
        } else {
            if (targetState != null) {
                turret.moveTo(targetState.addSwivelOffset(swivelOffset));
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
        SmartDashboard.putNumber("Swivel.position", Subsystems.turret.getPosition().swivelAngle());
        SmartDashboard.putNumber("ExtendRetract.position", Subsystems.turret.getPosition().extensionPosition());
        SmartDashboard.putNumber("Navx.yaw", navx.getAngle());
        SmartDashboard.putNumber("Navx.pitch", navx.getPitch());
    }

}
