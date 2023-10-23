package frc.robot.controls.controllers;

/**
 * <p>
 * An abstraction layer for controller types. Pretty much useless now that we
 * have a driver station that we're keeping consistent and not just whatever 2
 * random controllers we pulled out of the Portal to the Cable Dimension.
 * 
 * <p>
 * Implemented by:
 * {@link frc.robot.controls.controllers.ThrustMasterController}
 */
public interface Controller {

    int getPort();

    ControllerType getControllerType();

    double getTriggerValue();

    boolean getTrigger();

    boolean getTriggerPressed();

    double getStickX();

    double getStickY();

    double getStickZ();

    boolean getButtonPressedById(int buttonId);

    boolean getButtonById(int buttonId);

    boolean getDPadUp();

    boolean getDPadRight();

    boolean getDPadDown();

    boolean getDPadLeft();

}
