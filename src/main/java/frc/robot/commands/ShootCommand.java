package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PneumaticsHandler;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command {

    private final Intake s_Intake = RobotContainer.s_Intake;
    private final Shooter s_Shooter = RobotContainer.s_Shooter;

    private final PneumaticsHandler h_pneumatics = RobotContainer.h_pneumatics;

    private final Timer timer = new Timer();

    public ShootCommand() {

    }

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled.
     */
    @Override
    public void initialize() {
        timer.restart();
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        s_Shooter.setSpeed(1);
        if (Math.abs(s_Shooter.getCanCoderPosition()) < 1
                && (s_Shooter.getRPS() < -Constants.Shooter.TARGET_SHOOTER_SPEED
                        || timer.hasElapsed(Constants.Shooter.SHOOT_DELAY))) {
            s_Intake.setSpeed(1);
            h_pneumatics.setShooterSolenoid(true);
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
        s_Intake.setSpeed(0);
        s_Shooter.setSpeed(0);
        h_pneumatics.setShooterSolenoid(false);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
       // return timer.hasElapsed(1);
       return false;
    }
}
