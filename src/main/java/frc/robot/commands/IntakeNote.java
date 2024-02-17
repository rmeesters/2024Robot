package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class IntakeNote extends Command {

    private final Intake s_Intake = RobotContainer.s_Intake;

    private final Timer timer = new Timer();

    private boolean active = false;

    /**
     * Intake note starts the intake motors, including the conveyer,
     * and runs them until a note is loaded or the command is run a second time.
     */
    public IntakeNote() {

    }

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled.
     */
    @Override
    public void initialize() {
        timer.restart();

        // Toggle whether command will run or stop
        active = !active;
        if (!active)
            cancel();
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        s_Intake.setSpeed(0.2);
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
        active = false;

        s_Intake.setSpeed(0);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        // Return true if note is visible
        return s_Intake.inRange(2);
    }
}
