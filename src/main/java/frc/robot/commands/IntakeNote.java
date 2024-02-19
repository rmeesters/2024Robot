package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeNote extends Command {

    private final Intake s_Intake = RobotContainer.s_Intake;
    private final Shooter s_Shooter = RobotContainer.s_Shooter;

    /**
     * Intake note starts the intake motors, including the conveyer,
     * until a note is detected in the shooter.
     */
    public IntakeNote() {

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
        s_Shooter.prepareIntake();
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        s_Intake.setSpeed(0.6);
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
    }

    @Override
    public boolean isFinished() {
        // Return true if note is visible
        return s_Intake.inRange(2);
    }
}
