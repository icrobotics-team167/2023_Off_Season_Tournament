package frc.robot.routines.auto;

import frc.robot.routines.Action;
import frc.robot.routines.auto.*;
import frc.robot.subsystems.turret.TurretPosition;
import frc.robot.routines.Routine;

import java.time.Duration;

import com.pathplanner.lib.auto.NamedCommands;

/**
 * An enum for selecting auto routines.
 */
public enum AutoRoutines {
        // ROUTINE TEMPLATE
        //
        // ROUTINE_ID("Routine Name", new Routine(new Action[] {
        // new NullAction(),
        // })),

        NOTHING("Nothing", new Routine(new Action[] {
                        new NullAction(),
        })),
        GO_FORWARD_2_METERS("Go Forward 2 meters", new Routine(new Action[] {
                new FollowPath("Go Forward 2 Meters"),
        })),
        GO_SIDEWAYS_2_METERS("Go Sideways 2 meters", new Routine(new Action[] {
                
                new FollowPath("Go Sideways 2 Meters"),
        })), 
        GO_FORWARD_2_METERS_AND_MOVE_ARM("Go Forward 2 meters and move arm", new Routine(new Action[] {
                new FollowPath("Go Forward 2 Meters", true).withTurret(TurretPosition.INTAKE),
                new MoveArm(TurretPosition.INITIAL),
                new FollowPath("Go Backwards 2 Meters")
        }));

        public String name;
        public Routine actions;

        AutoRoutines(String name, Routine actions) {
                this.name = name;
                this.actions = actions;
        }
}