package frc.robot.subsystems;

import com.kauailabs.navx.frc.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drive.SwerveDriveBase;

public class Subsystems {

    public static final SwerveDriveBase driveBase;
    public static final AHRS navx;
    // public static final LimeLight limeLight;

    static {
        driveBase = SwerveDriveBase.getInstance();
        navx = new AHRS(SPI.Port.kMXP);
        // limeLight = LimeLight.getInstance();
    }

    public static void setInitialStates() {
    }

}
