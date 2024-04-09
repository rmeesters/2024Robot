package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PneumaticsHandler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.Shooter;

public class ReverseEverythingCommand extends Command {

    private final Intake s_Intake = RobotContainer.s_Intake;
    private final Roller s_Roller = RobotContainer.s_Roller;
    private final Shooter s_Shooter = RobotContainer.s_Shooter;

    private final PneumaticsHandler h_pneumatics = RobotContainer.h_pneumatics;

    public ReverseEverythingCommand() {
        
    }

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled.
     */
    @Override
    public void initialize() {
        h_pneumatics.setShooterSolenoid(true);
        h_pneumatics.setTiltSolenoid(true);
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        s_Shooter.setSpeed(-0.5);
        s_Intake.setSpeed(-0.5);
        s_Roller.setSpeed(0.3);
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
        s_Shooter.setSpeed(0);
        s_Intake.setSpeed(0);
        s_Roller.setSpeed(0);
        h_pneumatics.setShooterSolenoid(false);
    }

    @Override
    public boolean isFinished() {
       return false;
    }
}
