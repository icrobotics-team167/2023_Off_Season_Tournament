package frc.robot.subsystems;

import com.kauailabs.navx.frc.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drive.SparkTankDriveBase;
import frc.robot.subsystems.drive.TankDriveBase;

public class Subsystems {

    public static final TankDriveBase driveBase;
    public static final AHRS navx;
    public static final LimeLight limeLight;

    static {
        driveBase = SparkTankDriveBase.getInstance();
        navx = new AHRS(SPI.Port.kMXP);
        limeLight = LimeLight.getInstance();
    }

    public static void setInitialStates() {
        driveBase.resetEncoders();
        driveBase.setHighGear();
    }

}
