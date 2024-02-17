package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FireNote extends Command {

    private static final double TIME_LIMIT = 3;

    private final Intake s_Intake = RobotContainer.s_Intake;
    private final Shooter s_Shooter = RobotContainer.s_Shooter;

    private final Timer timer = new Timer();

    public FireNote() {

    }

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled.
     */
    @Override
    public void initialize() {
        // Start timer to start time limit
        timer.restart();
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        s_Intake.setSpeed(0.2);
        s_Shooter.setSpeed(1);
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
        s_Shooter.setSpeed(0);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(TIME_LIMIT);
    }
}
