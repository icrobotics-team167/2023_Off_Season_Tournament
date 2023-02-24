package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.controls.controllers.*;
import frc.robot.controls.controlschemes.*;
import frc.robot.routines.Action;
import frc.robot.routines.auto.*;
import frc.robot.routines.Teleop;
import frc.robot.subsystems.Subsystems;
import java.time.Duration;

public class Robot extends TimedRobot {

    private SendableChooser<AutoRoutines> autoChooser = new SendableChooser<>();
    private ControlScheme controls;
    private Action auto;
    private Teleop teleop;
    private Compressor phCompressor;

    public Robot() {
        super(Config.Settings.CPU_PERIOD);
    }

    @Override
    public void robotInit() {
        // Auto routine chooser
        // ROUTINE TEMPLATE:
        // autoChooser.addOption(AutoRoutines.(ROUTINE).name, AutoRoutines.(ROUTINE));
        autoChooser.setDefaultOption(AutoRoutines.SCORE_CONE.name, AutoRoutines.SCORE_CONE);
        autoChooser.addOption(AutoRoutines.SCORE_CUBE.name, AutoRoutines.SCORE_CUBE);
        autoChooser.addOption(AutoRoutines.BALANCE.name, AutoRoutines.BALANCE);
        autoChooser.addOption(AutoRoutines.GO_STRAIGHT.name, AutoRoutines.GO_STRAIGHT);
        autoChooser.addOption(AutoRoutines.NOTHING.name, AutoRoutines.NOTHING);
        autoChooser.addOption(AutoRoutines.TEST_PIVOT_ARM.name, AutoRoutines.TEST_PIVOT_ARM);
        autoChooser.addOption(AutoRoutines.TEST_RESET_ARM.name, AutoRoutines.TEST_RESET_ARM);
        SmartDashboard.putData("Autonomous Routines", autoChooser);

        Controller primaryController = null;
        switch (Config.Settings.PRIMARY_CONTROLLER_TYPE) {
            case XB:
                primaryController = new XBController(Config.Ports.PRIMARY_CONTROLLER);
                break;
            case PS:
                primaryController = new PSController(Config.Ports.PRIMARY_CONTROLLER);
                break;
            case NONE:
                primaryController = null;
                break;
        }
        Controller secondaryController = null;
        switch (Config.Settings.SECONDARY_CONTROLLER_TYPE) {
            case XB:
                secondaryController = new XBController(Config.Ports.SECONDARY_CONTROLLER);
                break;
            case PS:
                secondaryController = new PSController(Config.Ports.SECONDARY_CONTROLLER);
                break;
            case NONE:
                secondaryController = null;
                break;
        }
        if (primaryController == null && secondaryController == null) {
            controls = new NullController();
        } else if (primaryController != null && secondaryController == null) {
            controls = new SingleController(primaryController);
        } else if (primaryController != null && secondaryController != null) {
            controls = new DoubleController(primaryController, secondaryController);
        } else {
            // Fallback
            // This should be unreachable in normal conditions
            // This could only occur if the secondary controller is configured but the
            // primary controller isn't
            controls = new NullController();
        }

        try {
            phCompressor = new Compressor(2, PneumaticsModuleType.REVPH);
            // phCompressor.enableAnalog(60, 65);
            phCompressor.disable();
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating compressor: " + ex.getMessage(), true);
        }

        Subsystems.setInitialStates();
        // ******************AUTO********************* */

        teleop = new Teleop(controls);
    }

    @Override
    public void robotPeriodic() {
    }

    @Override
    public void autonomousInit() {
        auto = autoChooser.getSelected().actions;
        Subsystems.driveBase.resetEncoders();
        Subsystems.driveBase.setHighGear();
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
