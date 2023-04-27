package frc.robot.controls.controlschemes;

/**
 * <p>
 * An abstraction layer for control schemes. See the other control scheme
 * classes for implementation details.
 * 
 * <p>
 * Implementation classes:
 * {@link frc.robot.controls.controlschemes.DeltaJoystickController}
 * {@link frc.robot.controls.controlschemes.DoubleController}
 * {@link frc.robot.controls.controlschemes.SingleController}
 * {@link frc.robot.controls.controlschemes.NullController}
 */
public abstract class ControlScheme {

    // Driving (Primary driver's controller)

    public abstract double getSwerveX();

    public abstract double getSwerveY();

    public abstract double getSwerveTurn();

    public abstract boolean doSlowMode();

    public abstract boolean toggleLimelight();
}
