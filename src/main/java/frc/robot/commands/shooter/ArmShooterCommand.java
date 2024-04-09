package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.PneumaticsHandler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ArmShooterCommand extends Command {

    private final Shooter s_Shooter = RobotContainer.s_Shooter;
    private final PneumaticsHandler h_pneumatics = RobotContainer.h_pneumatics;

    public ArmShooterCommand() {
        
    }

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled.
     */
    @Override
    public void initialize() {
        s_Shooter.setShaftRotation(Constants.Shooter.SHOOT_POSITION);
        h_pneumatics.setTiltSolenoid(false);

        RobotContainer.shooterTimer.restart();
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
        s_Shooter.setShaftRotation(Constants.Shooter.MOVE_POSITION);
        s_Shooter.setSpeed(0);
        h_pneumatics.setTiltSolenoid(true);
        
        RobotContainer.shooterTimer.stop();
    }

    @Override
    public boolean isFinished() {
       return false;
    }
}
