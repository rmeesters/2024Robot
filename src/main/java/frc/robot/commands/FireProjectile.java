package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FireProjectile extends Command {

    private final Shooter s_Shooter = RobotContainer.s_Shooter;
    private final Intake s_Intake = RobotContainer.s_Intake;

    private final Timer timer = new Timer();

    public FireProjectile() {

    }

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled.
     */
    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Projectile Fired", false);

        // Start timer to create ordered sequences
        timer.restart();
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        s_Shooter.setShooterSpeed(1, false);
        // After 1 second, start the conveyer belt
        if (timer.hasElapsed(1)) {
            s_Intake.setSpeed(1, true);
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
        timer.stop();
        s_Shooter.setShooterSpeed(0, false);
        s_Intake.setSpeed(0, true);
        SmartDashboard.putBoolean("Projectile Fired", true);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2);
    }
}

