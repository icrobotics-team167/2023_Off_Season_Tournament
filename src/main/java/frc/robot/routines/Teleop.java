package frc.robot.routines;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config;
import frc.robot.controls.controlschemes.ControlScheme;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.SwerveDriveBase;

public class Teleop {
    private ControlScheme controls;
    private SwerveDriveBase driveBase;

    private LimeLight limeLight;
    private AHRS navx = Subsystems.navx;

    public Teleop(ControlScheme controls) {
        this.controls = controls;
        driveBase = Subsystems.driveBase;

        limeLight = LimeLight.getInstance();
    }

    public void init() {
        driveBase.resetPosition();

    }

    public void periodic() {

        if (controls.toggleLimelight()) {
            limeLight.toggleMode();
        }

        SmartDashboard.putNumber("Navx.yaw", navx.getAngle());
        SmartDashboard.putNumber("Navx.pitch", navx.getPitch());

    }

}
