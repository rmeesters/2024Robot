package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

public class MoveCommand extends Command {

    private final Swerve s_Swerve = RobotContainer.s_Swerve;
    private final Trajectory trajectory;
    private final SwerveControllerCommand swerveControllerCommand;

    public MoveCommand(List<Pose2d> points, boolean reversed) {
        TrajectoryConfig config = new TrajectoryConfig(
                Constants.Autos.MAX_SPEED_IN_METERS_PER_SECOND,
                Constants.Autos.MAX_ACCELERATION_IN_METERS_PER_SECOND_SQUARED)
                .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

        ProfiledPIDController thetaController = new ProfiledPIDController(
                Constants.Autos.CONTROLLER_ANGLE, 0, 0, Constants.Autos.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        trajectory = TrajectoryGenerator.generateTrajectory(points, config);

        swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.Autos.CONTROLLER_PX, 0, 0),
                new PIDController(Constants.Autos.CONTROLLER_PY, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
    }

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled.
     */
    @Override
    public void initialize() {
        s_Swerve.setPose(trajectory.getInitialPose());
        swerveControllerCommand.initialize();
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        swerveControllerCommand.execute();
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
        swerveControllerCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return swerveControllerCommand.isFinished();
    }

}
