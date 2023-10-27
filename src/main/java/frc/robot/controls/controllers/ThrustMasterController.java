package frc.robot.controls.controllers;

import edu.wpi.first.wpilibj.Joystick;

/**
 * A control scheme consisting of 4 flight sim joysticks.
 */
public class ThrustMasterController implements Controller {
    Joystick joystick;
    private int port;

    /**
     * Constructs a flightsim joystick.
     */
    public ThrustMasterController(int port) {
        joystick = new Joystick(port);
        this.port = port;
    }

    @Override
    public int getPort() {
        return port;
    }

    @Override
    public ControllerType getControllerType() {
        return ControllerType.JOYSTICK;
    }

    @Override
    public double getTriggerValue() {

        return 0;
    }

    @Override
    public boolean getTrigger() {
        return joystick.getTrigger();
    }

    @Override
    public boolean getTriggerPressed() {
        return joystick.getTriggerPressed();
    }

    @Override
    public double getStickX() {
        return -joystick.getX();
    }

    @Override
    public double getStickY() {

        return -joystick.getY();
    }

    @Override
    public double getStickZ() {
        return joystick.getZ();
    }

    @Override
    public boolean getDPadUp() {
        return joystick.getPOV() == 0;
    }

    @Override
    public boolean getDPadRight() {
        return joystick.getPOV() == 90;
    }

    @Override
    public boolean getDPadDown() {
        return joystick.getPOV() == 180;
    }

    @Override
    public boolean getDPadLeft() {
        return joystick.getPOV() == 270;
    }

    @Override
    public boolean getButtonById(int buttonId) {
        return joystick.getRawButton(buttonId);
    }

    @Override
    public boolean getButtonPressedById(int buttonId) {
        return joystick.getRawButtonPressed(buttonId);
    }

}
