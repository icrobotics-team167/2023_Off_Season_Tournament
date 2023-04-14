package frc.robot.controls.controlschemes;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config;
import frc.robot.controls.controllers.Controller;

public class DoubleController extends ControlScheme {

    private Controller primary;
    private Controller secondary;

    public DoubleController(Controller primary, Controller secondary) {
        this.primary = primary;
        this.secondary = secondary;
    }

    // Drive

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
        return primary.getRightBumperToggled();
    }

}