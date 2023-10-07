package frc.robot.routines.auto;

/**
 * An enum to select states for autonomous actions.
 */
public enum AutoState {
    INIT, // Runs init(). Moves to PERIODIC after that.
    PERIODIC, // Runs periodic() every robot tick. Will move to DONE if isDone() == true (checked every tick)
    DONE, // Runs done(). Moves to FINISH after that.
    FINISHED, // Exits the code, calls the next Action in the routine if there is one.
}
