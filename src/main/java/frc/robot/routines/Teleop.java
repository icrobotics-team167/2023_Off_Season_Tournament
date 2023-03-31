package frc.robot.routines;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config;
import frc.robot.controls.controlschemes.ControlScheme;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.TankDriveBase;

public class Teleop {
    private ControlScheme controls;
    private TankDriveBase driveBase;

    private LimeLight limeLight;
    private AHRS navx = Subsystems.navx;

    public Teleop(ControlScheme controls) {
        this.controls = controls;
        driveBase = Subsystems.driveBase;

        limeLight = LimeLight.getInstance();
    }

    public void init() {
        driveBase.resetEncoders();

    }

    public void periodic() {
        if (controls.doSwitchHighGear()) {
            driveBase.setHighGear();
        } else if (controls.doSwitchLowGear()) {
            driveBase.setLowGear();
        }
        if (Config.Settings.TANK_DRIVE) {
            driveBase.tankDrive(controls.getTankLeftSpeed(),
                    controls.getTankRightSpeed());
        } else {
            driveBase.arcadeDrive(controls.getArcadeThrottle() * 0.8,
                    controls.getArcadeWheel() * .5);
        }

        if (controls.toggleLimelight()) {
            limeLight.toggleMode();
        }

        SmartDashboard.putNumber("Navx.yaw", navx.getAngle());
        SmartDashboard.putNumber("Navx.pitch", navx.getPitch());

    }

}
