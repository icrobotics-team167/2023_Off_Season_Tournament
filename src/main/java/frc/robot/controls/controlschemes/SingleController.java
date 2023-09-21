package frc.robot.controls.controlschemes;

import frc.robot.Config;
import frc.robot.controls.controllers.Controller;

/**
 * <p>
 * A control scheme with 1 game controller.
 * <p>
 * Does not have secondary driver functionality due to a lack of buttons.
 */
public class SingleController extends ControlScheme {

    private Controller primary;

    /**
     * Constructs a new control scheme with 1 controller.
     * 
     * @param controller The controller.
     */
    public SingleController(Controller controller) {
        primary = controller;
    }

    @Override
    public double getSwerveX() {
        return primary.getLeftStickX() >= Config.Tolerances.PRIMARY_CONTROLLER_DEADZONE_SIZE ? primary.getLeftStickX()
                : 0;
    }

    @Override
    public double getSwerveY() {
        return primary.getLeftStickY() >= Config.Tolerances.PRIMARY_CONTROLLER_DEADZONE_SIZE ? primary.getLeftStickY()
                : 0;
    }

    @Override
    public double getSwerveTurn() {
        return primary.getRightStickX() >= Config.Tolerances.PRIMARY_CONTROLLER_DEADZONE_SIZE ? primary.getRightStickX()
                : 0;
    }

    @Override
    public boolean doSlowMode() {
        return primary.getLeftTrigger();
    }

    @Override
    public boolean toggleLimelight() {
        return false;
    }

}
