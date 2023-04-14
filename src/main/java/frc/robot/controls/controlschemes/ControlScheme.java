package frc.robot.controls.controlschemes;

public abstract class ControlScheme {

    // Driving (Primary controller)

    public abstract double getSwerveX();

    public abstract double getSwerveY();

    public abstract double getSwerveTurn();

    public abstract boolean doSlowMode();

    public abstract boolean toggleLimelight();
}
