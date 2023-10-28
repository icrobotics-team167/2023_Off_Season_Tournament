package frc.robot.routines.auto;

import frc.robot.routines.Action;
import frc.robot.routines.auto.*;
import frc.robot.subsystems.turret.TurretPosition;
import frc.robot.routines.Routine;

import java.time.Duration;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

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
        THE_ONE_PIECE_IS_REAL("One piece auto", new Routine(new Action[] {
                        new SetPos(new Pose2d(1.9, 5.0, Rotation2d.fromDegrees(180))),
                        new MoveArm(TurretPosition.CONE_HIGH),
                        new Outtake(),
                        new MoveArm(TurretPosition.INITIAL),
                        new SimpleDrive(new ChassisSpeeds(1.5, 0, 0), 3)
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