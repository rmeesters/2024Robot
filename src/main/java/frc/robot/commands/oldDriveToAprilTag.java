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
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class oldDriveToAprilTag extends Command {

    private final Swerve s_Swerve = RobotContainer.s_Swerve;

    private final Timer timer = new Timer();

    // Poses in trajectory
    private List<Pose2d> points;

    // Movement variables
    private Trajectory trajectory;
    private Supplier<Pose2d> pose;
    private SwerveDriveKinematics kinematics;
    private HolonomicDriveController controller;
    private Consumer<SwerveModuleState[]> outputModuleStates;
    private Supplier<Rotation2d> desiredRotation;

    private boolean APRILTAG_IS_VISIBLE;

    @Deprecated
    public oldDriveToAprilTag() {

    }

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled.
     */
    @Override
    public void initialize() {
        // Stop if apriltag is visible
        APRILTAG_IS_VISIBLE = LimelightHelpers.getTV(Constants.Limelight.Back.NAME);
        if (!APRILTAG_IS_VISIBLE) {
            System.out.println("No apriltag visible");
            cancel();
            return;
        }

        // Create trajectory
        calculatePoints();
        setupSwerveController();

        SmartDashboard.putNumber("Limelight X Dest", points.get(1).getX());
        SmartDashboard.putNumber("Limelight Y Dest", points.get(1).getY());
        SmartDashboard.putBoolean("Limelight Running", true);

        // Start timer to finish command after arival
        timer.restart();

        // Reset postion
        s_Swerve.setPose(points.get(0));
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        var desiredState = trajectory.sample(timer.get());

        var targetChassisSpeeds = controller.calculate(pose.get(), desiredState, desiredRotation.get());
        var targetModuleStates = kinematics.toSwerveModuleStates(targetChassisSpeeds);

        outputModuleStates.accept(targetModuleStates);
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
        if (!APRILTAG_IS_VISIBLE) {
            return;
        }

        timer.stop();
        SmartDashboard.putBoolean("Limelight Running", false);
    }

    @Override
    public boolean isFinished() {
        return !APRILTAG_IS_VISIBLE || timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    private void calculatePoints() {
        double tx = LimelightHelpers.getTX(Constants.Limelight.Back.NAME);
        double angled_ty = LimelightHelpers.getTY(Constants.Limelight.Back.NAME);
        double ty = Constants.Limelight.Back.CAMERA_ANGLE + angled_ty;

        double dx = (Constants.Limelight.Pipelines.Speaker.APRILTAG_HEIGHT - Constants.Limelight.Back.CAMERA_HEIGHT)
                / Math.tan(Math.toRadians(ty));
        double dy = dx * Math.tan(Math.toRadians(tx));

        // double hyp = Math.sqrt(dx * dx + dy * dy);

        // double ratio = (hyp - Constants.Limelight.TARGET_DISTANCE_AWAY) / hyp;

        // dx *= -ratio;
        // dy *= ratio;

        dy -= 1; //TODO onstants.Limelight.TARGET_TAG_DISTANCE;

        points = List.of(
                new Pose2d(0, 0, new Rotation2d(0)),
                new Pose2d(dx, dy, new Rotation2d(0)));
    }

    private void setupSwerveController() {
        TrajectoryConfig config = new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics).setReversed(true);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

        trajectory = TrajectoryGenerator.generateTrajectory(points, config);
        pose = s_Swerve::getPose;
        kinematics = Constants.Swerve.swerveKinematics;
        controller = new HolonomicDriveController(
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController);
        desiredRotation = () -> trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation();
        outputModuleStates = s_Swerve::setModuleStates;
        addRequirements(s_Swerve);
    }
}
