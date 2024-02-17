package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootNote extends Command {

    private static final double TIME_LIMIT = 1;

    private final Intake s_Intake = RobotContainer.s_Intake;
    private final Shooter s_Shooter = RobotContainer.s_Shooter;

    private final Timer timer = new Timer();

    private Integer targetAngle = null;

    private double travelTime = 0;

    public ShootNote() {

    }

    public ShootNote atAngle(int angle) {
        targetAngle = angle;
        return this;
    }

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled.
     */
    @Override
    public void initialize() {
        timer.restart();

        if (targetAngle != null) {
            travelTime = s_Shooter.setAngle(targetAngle);
        }
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        if (targetAngle == null) {
            s_Shooter.setSpeed(1);
            s_Intake.setSpeed(0.2);
            return;
        }

        if (timer.hasElapsed(travelTime - 1)) {
            s_Shooter.setSpeed(1);

            if (timer.hasElapsed(travelTime)) {
                s_Intake.setSpeed(0.2);
            }
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
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        if (targetAngle == null)
            return timer.hasElapsed(TIME_LIMIT);
        return timer.hasElapsed(TIME_LIMIT + travelTime);
    }
}
