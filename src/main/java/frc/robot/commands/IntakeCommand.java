package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
//import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter;

public class IntakeCommand extends Command {


    private final Intake s_Intake = RobotContainer.s_Intake;
    private final Shooter s_Shooter = RobotContainer.s_Shooter;

    /**
     * Intake note starts the intake motors, including the conveyer,
     * until a note is detected in the shooter.
     */
    public IntakeCommand() {

    }

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled.
     */
    @Override
    public void initialize() {
        if (isFinished()) {
            cancel();
            return;
        }
        s_Shooter.setShaftRotation(Constants.Shooter.PICKUP_POSITION);
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        //if (s_Intake.inRange(2)) RobotContainer.noteLoaded = true;
        s_Intake.setSpeed(1);
    }

    /**
     * The action to take when the command ends.
     * Called either when the command finishes normally or interrupted/canceled.
     *
     * Do not schedule commands here that share requirements with this command.
     * Use {@link #andThen(Command...)} instead.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        s_Intake.setSpeed(0);
        s_Shooter.setShaftRotation(Constants.Shooter.MOVE_POSITION);
    }

    @Override
    public boolean isFinished() {
        // Return true if note is visible
        return s_Intake.inRange(2);
    }
}
