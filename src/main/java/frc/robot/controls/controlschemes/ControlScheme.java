package frc.robot.controls.controlschemes;

public abstract class ControlScheme {

    // Driving (Primary controller)

    // Tank Drive
    public abstract double getTankLeftSpeed();

    public abstract double getTankRightSpeed();

    // Arcade Drive
    public abstract double getArcadeThrottle();

    public abstract double getArcadeWheel();

    // Gearing
    public abstract boolean doSwitchLowGear();

    public abstract boolean doSwitchHighGear();

    public abstract boolean doSlowMode();

    public abstract boolean toggleLimelight();
}
