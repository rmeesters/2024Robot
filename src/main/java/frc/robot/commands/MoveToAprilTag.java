package frc.robot.commands;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Timer;

public class MoveToAprilTag extends Command {

    private final Swerve s_Swerve = RobotContainer.s_Swerve;

    private final TrajectoryConfig config = new TrajectoryConfig(
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
                    Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

    private List<Pose2d> points;

    private Timer m_timer;
    private Trajectory m_trajectory;
    private Supplier<Pose2d> m_pose;
    private SwerveDriveKinematics m_kinematics;
    private HolonomicDriveController m_controller;
    private Consumer<SwerveModuleState[]> m_outputModuleStates;
    private Supplier<Rotation2d> m_desiredRotation;

    public MoveToAprilTag() {
    }

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled.
     */
    @Override
    public void initialize() {
        // Check if apriltag exists
        boolean APRILTAG_IS_VISIBLE = LimelightHelpers.getTV(Constants.Limelight.NAME);
        if (!APRILTAG_IS_VISIBLE) {
            cancel();
        }
        
        // Create trajectory
        calculatePoints();
        setupSwerveController();

        m_timer = new Timer();
        m_timer.restart();
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        var desiredState = m_trajectory.sample(m_timer.get());
    
        var targetChassisSpeeds = m_controller.calculate(m_pose.get(), desiredState, m_desiredRotation.get());
        var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);
    
        m_outputModuleStates.accept(targetModuleStates);
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
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
      return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }

    private void calculatePoints() {

        double tx = LimelightHelpers.getTX(Constants.Limelight.NAME);
        double angled_ty = LimelightHelpers.getTY(Constants.Limelight.NAME);

        double ty = Math.toRadians(Constants.Limelight.CAMERA_ANGLE + angled_ty);

        double dx = (Constants.Dimensions.APRILTAG_HEIGHT - Constants.Dimensions.CAMERA_HEIGHT) / Math.tan(ty);
        double dy = dx * Math.tan(tx);

        double hyp = Math.sqrt( dx*dx + dy*dy );

        double ratio = (hyp - 1) / hyp;

        dx *= ratio;
        dy *= ratio;

        points = List.of(
            new Pose2d(0, 0, new Rotation2d(0)),
            new Pose2d(dx, dy, new Rotation2d(0)));
    }

    private void setupSwerveController() {
        m_trajectory = TrajectoryGenerator.generateTrajectory(points, config);
        m_pose = s_Swerve::getPose;
        m_kinematics = Constants.Swerve.swerveKinematics;
        m_controller = new HolonomicDriveController(
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController);
        m_desiredRotation = () -> m_trajectory.getStates().get(m_trajectory.getStates().size() - 1).poseMeters.getRotation();
        m_outputModuleStates = s_Swerve::setModuleStates;
        addRequirements(s_Swerve);
    }
}
