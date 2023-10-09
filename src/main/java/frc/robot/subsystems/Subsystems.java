package frc.robot.subsystems;

import frc.robot.subsystems.drive.SwerveDriveBase;
import frc.robot.subsystems.turret.*;

/**
 * Initializes robot subsystems.
 */
public class Subsystems {

    public static final SwerveDriveBase driveBase;
    public static final Gyro gyro;
    public static final LimeLight limeLight;
    public static final Turret turret;
    public static final Claw claw;

    static {
        driveBase = SwerveDriveBase.getInstance();
        gyro = Gyro.getInstance();
        limeLight = LimeLight.getInstance();
        turret = Turret.getInstance();
        claw = Claw.getInstance();
    }

    public static void setInitialStates() {
        gyro.reset();
        driveBase.resetPosition();
    }

}
