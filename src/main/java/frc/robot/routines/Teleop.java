package frc.robot.routines;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.controls.controlschemes.ControlScheme;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.SwerveDriveBase;

public class Teleop {
    private ControlScheme controls;
    private SwerveDriveBase driveBase;

    // TODO: Figure out max movement speeds
    private final double MAX_MOVE_SPEED = 1.0; // m/s;
    private final double MAX_TURN_SPEED = Units.rotationsToRadians(1); // rotations/s
    private final double SLOWMODE_MULT = 0.5; // 50%

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

        double moveSpeed = controls.doSlowMode() ? MAX_MOVE_SPEED * SLOWMODE_MULT : MAX_MOVE_SPEED;
        double turnSpeed = controls.doSlowMode() ? MAX_TURN_SPEED * SLOWMODE_MULT : MAX_TURN_SPEED;
        Subsystems.driveBase.drive(controls.getSwerveX() * moveSpeed, controls.getSwerveY() * moveSpeed,
                controls.getSwerveTurn() * turnSpeed);

        SmartDashboard.putNumber("Navx.yaw", navx.getAngle());
        SmartDashboard.putNumber("Navx.pitch", navx.getPitch());

    }

}
