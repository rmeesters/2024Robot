package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PneumaticsHandler;
import frc.robot.subsystems.Roller;

public class ScoreAmpCommand extends Command {

    private final Roller s_Roller = RobotContainer.s_Roller;

    private final PneumaticsHandler h_pneumatics = RobotContainer.h_pneumatics;

    private final Timer timer = new Timer();

    private final double ROLLER_DELAY = 0.2;

    public ScoreAmpCommand() {

    }

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled.
     */
    @Override
    public void initialize() {
        h_pneumatics.setTiltSolenoid(false);

        timer.restart();
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        // Delayed
        if (timer.hasElapsed(ROLLER_DELAY)) {
            s_Roller.setSpeed(1);
        }
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
        s_Roller.setSpeed(0);
        h_pneumatics.setTiltSolenoid(true);

        timer.stop();
    }

    @Override
    public boolean isFinished() {
       // return timer.hasElapsed(1);
       return false;
    }
}
