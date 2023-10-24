package frc.robot.routines.auto;

import frc.robot.routines.Action;
import frc.robot.routines.auto.*;
import frc.robot.subsystems.turret.TurretPosition;
import frc.robot.routines.Routine;

import java.time.Duration;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

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
                        new FollowPath("Go Forward 2 Meters").withTurret(TurretPosition.INTAKE),
                        new MoveArm(TurretPosition.INITIAL),
                        new FollowPath("Go Backwards 2 Meters")
        })),
        SCORE_CONE_THEN_CUBE("Score Cone then Cube", new Routine(new Action[] {
                        new SetPos(new Pose2d(1.9, 5.0, Rotation2d.fromDegrees(180))),
                        new MoveArm(TurretPosition.CONE_HIGH),
                        new Outtake(),
                        new FollowPath("Score then back up pt 1", new Pose2d(1.9, 5.0, Rotation2d.fromDegrees(-179.9)))
                                        .withTurret(TurretPosition.INITIAL),
                        new FollowPath("Score then back up pt 2").withTurret(TurretPosition.INTAKE),
                        new FollowPath("Score then back up pt 3").withIntake(),
                        new FollowPath("Score then back up pt 4").withTurret(TurretPosition.INITIAL),
                        new MoveArm(TurretPosition.CUBE_HIGH),
                        new Outtake(),
        }));

        public String name;
        public Routine actions;

        AutoRoutines(String name, Routine actions) {
                this.name = name;
                this.actions = actions;
        }
}