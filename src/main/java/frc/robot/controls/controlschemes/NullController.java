package frc.robot.controls.controlschemes;

/**
 * A control scheme where no controller is attached.
 */
public class NullController extends ControlScheme {
    /**
     * Constructs a new control scheme with no controllers.
     */
    public NullController() {

    }

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
