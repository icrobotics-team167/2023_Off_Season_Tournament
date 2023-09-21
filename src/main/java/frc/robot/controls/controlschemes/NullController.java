package frc.robot.controls.controlschemes;

/**
 * A control scheme where no controller is attached. Fallback.
 */
public class NullController extends ControlScheme {

    @Override
    public boolean doSlowMode() {

        return false;
    }

    @Override
    public boolean toggleLimelight() {
        return false;
    }

    @Override
    public double getSwerveX() {
        return 0;
    }

    @Override
    public double getSwerveY() {
        return 0;
    }

    @Override
    public double getSwerveTurn() {
        return 0;
    }
}
