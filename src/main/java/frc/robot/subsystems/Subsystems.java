package frc.robot.subsystems;

import com.kauailabs.navx.frc.*;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.subsystems.drive.SwerveDriveBase;

/**
 * Initializes robot subsystems.
 */
public class Subsystems {

    public static final SwerveDriveBase driveBase;
    public static final AHRS navx;
    public static final LimeLight limeLight;
    public static final Turret turret;
    public static final Claw claw;

    static {
        driveBase = SwerveDriveBase.getInstance();
        navx = new AHRS(SPI.Port.kMXP);
        limeLight = LimeLight.getInstance();
        turret = Turret.getInstance();
        claw = Claw.getInstance();
    }

    public static void setInitialStates() {
        navx.reset();
    }

}
