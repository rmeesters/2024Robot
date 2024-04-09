package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.PneumaticsHandler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.Shooter;

public class PrepareAmpCommand extends Command {

    private final Intake s_Intake = RobotContainer.s_Intake;
    private final Shooter s_Shooter = RobotContainer.s_Shooter;
    private final Roller s_Roller = RobotContainer.s_Roller;

    private final PneumaticsHandler h_pneumatics = RobotContainer.h_pneumatics;

    public PrepareAmpCommand() {

    }

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled.
     */
    @Override
    public void initialize() {
        s_Shooter.setShaftRotation(Constants.Shooter.VERTICAL_POSITION);
        h_pneumatics.setTiltSolenoid(true);
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        // Delayed
        if (Math.abs(s_Shooter.getCanCoderPosition() - Constants.Shooter.VERTICAL_POSITION) < 5) {
            s_Intake.setSpeed(1);
            h_pneumatics.setShooterSolenoid(true);
            return;
        }

        // Immediate
        s_Roller.setSpeed(-0.3);
        s_Shooter.setSpeed(0.1);
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
        s_Intake.setSpeed(0);
        s_Shooter.setSpeed(0);
        s_Roller.setSpeed(0);
        h_pneumatics.setShooterSolenoid(false);
    }

    @Override
    public boolean isFinished() {
       // return timer.hasElapsed(1);
       return false;
    }
}
