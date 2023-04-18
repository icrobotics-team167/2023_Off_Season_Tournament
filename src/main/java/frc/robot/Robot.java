package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.controls.controllers.*;
import frc.robot.controls.controlschemes.*;
import frc.robot.routines.Action;
import frc.robot.routines.Routine;
import frc.robot.routines.auto.*;
import frc.robot.routines.Teleop;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Subsystems;
import java.time.Duration;

public class Robot extends TimedRobot {

    private SendableChooser<AutoRoutines> autoChooser = new SendableChooser<>();
    private ControlScheme controls;
    private Action auto;
    private Teleop teleop;
    private LimeLight limeLight;

    public Robot() {
        super(Config.Settings.CPU_PERIOD);
    }

    @Override
    public void robotInit() {
        // Initialize the auto routine selector.
        AutoRoutines defaultRoutine = AutoRoutines.NOTHING; // Default routine is nothing to avoid accidents
        // Loop through the routines enum and add each of them to the routine selector.
        for (AutoRoutines routine : AutoRoutines.values()) {
            if (routine != defaultRoutine) {
                autoChooser.addOption(routine.name, routine);
            } else {
                autoChooser.setDefaultOption(defaultRoutine.name, defaultRoutine);
            }

        }
        SmartDashboard.putData("Autonomous Routines", autoChooser);

        Controller primaryController = null;
        switch (Config.Settings.PRIMARY_CONTROLLER_TYPE) {
            case JOYSTICK:
                primaryController = new ThrustMasterController(Config.Ports.PRIMARY_CONTROLLER);
                break;
            case NONE:
                primaryController = null;
                break;
        }
        Controller secondaryController = null;
        switch (Config.Settings.SECONDARY_CONTROLLER_TYPE) {
            case JOYSTICK:
                secondaryController = new ThrustMasterController(Config.Ports.SECONDARY_CONTROLLER);
                break;
            case NONE:
                secondaryController = null;
                break;
        }

        Controller tertiaryController = null;
        if (Config.Settings.TERTIARY_CONTROLLER_TYPE == ControllerType.JOYSTICK) {
            tertiaryController = new ThrustMasterController(Config.Ports.TERTIARY_CONTROLLER);
        }

        Controller quaternaryController = null;
        if (Config.Settings.QUATERNARY_CONTROLLER_TYPE == ControllerType.JOYSTICK) {
            quaternaryController = new ThrustMasterController(Config.Ports.QUATERNARY_CONTROLLER);
        }

        if (primaryController == null && secondaryController == null) {
            controls = new NullController();
        } else if (primaryController != null && secondaryController == null) {
            controls = new SingleController(primaryController);
        } else if (Config.Settings.PRIMARY_CONTROLLER_TYPE == ControllerType.JOYSTICK) {
            // If the first contorller is a JOYSTICK type, assume we have four joysticks.
            controls = new DeltaJoystickController(primaryController, secondaryController, tertiaryController,
                    quaternaryController);
        } else if (primaryController != null && secondaryController != null) {
            controls = new DoubleController(primaryController, secondaryController);
        } else {
            // Fallback
            // This should be unreachable in normal conditions
            // This could only occur if the secondary controller is configured but the
            // primary controller isn't
            controls = new NullController();
        }

        limeLight = LimeLight.getInstance();

        Subsystems.setInitialStates();
        teleop = new Teleop(controls);
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("Robot.batteryVoltage", RobotController.getBatteryVoltage());
    }

    @Override
    public void autonomousInit() {
        auto = autoChooser.getSelected().actions;
        limeLight.setVisionMode();
        Subsystems.driveBase.resetPosition();
        auto.exec();
        // System.out.println("Auto selected: " + autoChooser.getSelected().name);
    }

    @Override
    public void autonomousPeriodic() {
        auto.exec();
    }

    @Override
    public void teleopInit() {
        teleop.init();
        limeLight.setCameraMode();
    }

    @Override
    public void teleopPeriodic() {
        teleop.periodic();
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

}
