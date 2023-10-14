package frc.robot.routines.auto;

import frc.robot.routines.Action;
import frc.robot.routines.auto.*;
import frc.robot.routines.Routine;

import java.time.Duration;

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
        FOLLOW_PATH_TEST("Follow Path Test", new Routine(new Action[] {
                new FollowPath("New Path", 1000),
        }));

        public String name;
        public Routine actions;

        AutoRoutines(String name, Routine actions) {
                this.name = name;
                this.actions = actions;
        }
}