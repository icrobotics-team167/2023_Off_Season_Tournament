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

    double getLeftTriggerValue();

    boolean getLeftTrigger();

    boolean getLeftTriggerPressed();

    boolean getLeftBumper();

    boolean getLeftBumperPressed();

    double getRightTriggerValue();

    boolean getRightTrigger();

    boolean getRightTriggerPressed();

    boolean getRightBumper();

    boolean getRightBumperToggled();

    double getLeftStickX();

    double getLeftStickY();

    double getLeftStickZ();

    boolean getLeftStickButton();

    boolean getLeftStickButtonPressed();

    double getRightStickX();

    double getRightStickY();

    boolean getRightStickButton();

    boolean getRightStickButtonPressed();

    boolean getButtonPressedById(int buttonId);

    boolean getButtonById(int buttonId);

    boolean getAButton();

    boolean getAButtonPressed();

    boolean getBButton();

    boolean getBButtonPressed();

    boolean getXButton();

    boolean getXButtonPressed();

    boolean getYButton();

    boolean getYButtonPressed();

    boolean getBackButton();

    boolean getBackButtonPressed();

    boolean getStartButton();

    boolean getStartButtonPressed();

    boolean getDPadUp();

    boolean getDPadRight();

    boolean getDPadDown();

    boolean getDPadLeft();

}
