package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autos.AutoDrive;

public class MoveCommand extends Command {

    AutoDrive autoDrive;

    public MoveCommand(List<Pose2d> points, boolean reverse) {
        autoDrive = new AutoDrive(points, reverse);
    }

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled.
     */
    @Override
    public void initialize() {
        autoDrive.initialize();
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        autoDrive.execute();
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
        autoDrive.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return autoDrive.isFinished();
    }
    
}
