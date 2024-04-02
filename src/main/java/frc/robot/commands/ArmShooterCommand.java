package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ArmShooterCommand extends Command {

    private final Shooter s_Shooter = RobotContainer.s_Shooter;

    public ArmShooterCommand() {
        
    }

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled.
     */
    @Override
    public void initialize() {
        s_Shooter.setShaftRotation(0);
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
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
        s_Shooter.setShaftRotation(Constants.Shooter.PICKUP_POSITION);
        s_Shooter.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
       return false;
    }
}
