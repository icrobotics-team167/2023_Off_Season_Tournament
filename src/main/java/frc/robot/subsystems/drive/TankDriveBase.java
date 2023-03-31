package frc.robot.subsystems.drive;

public interface TankDriveBase {

    // Teleop drive
    void tankDrive(double leftSpeed, double rightSpeed);

    void arcadeDrive(double throttle, double wheel);

    // Gearing
    void toggleGearing();

    void setHighGear();

    void setLowGear();

    boolean isHighGear();

    boolean isLowGear();

    // Auto
    void stop();

    double getLeftEncoderPosition();

    double getRightEncoderPosition();

    void resetEncoders();

    void setCoast();

    void setBrake();

    // Utility
    double metersPerSecondToRPM(double metersPerSecond);
}
