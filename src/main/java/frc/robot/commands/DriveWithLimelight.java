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
import frc.robot.autos.AutoDrive;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveWithLimelight extends Command {

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

    private boolean TARGET_IS_VISIBLE;

    private AutoDrive auto;
    private boolean isAprilTag;

    /**
     * Drive to target using limelight
     * 
     * @param isAprilTag True: apriltag False: note
     */
    public DriveWithLimelight(boolean isAprilTag) {
        this.isAprilTag = isAprilTag;
    }

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled.
     */
    @Override
    public void initialize() {
        // Stop if target is not visible
        TARGET_IS_VISIBLE = LimelightHelpers.getTV(Constants.Limelight.NAME);
        if (!TARGET_IS_VISIBLE) {
            System.out.println("No target visible");
            //cancel(); //TODO Test this
            return;
        }

        List<Pose2d> points = isAprilTag ? calculateAprilTagPoints() : calculateNotePoints();
        auto = new AutoDrive(points, isAprilTag);
        auto.initialize();
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        if (!TARGET_IS_VISIBLE) {
            System.out.println("Caught by execute");
            return;
        }

        auto.execute();
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
        if (!TARGET_IS_VISIBLE) {
            System.out.println("Caught by end");
            return;
        }

        auto.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return !TARGET_IS_VISIBLE || auto.isFinished();
    }

    private List<Pose2d> calculateAprilTagPoints() {
        double tx = LimelightHelpers.getTX(Constants.Limelight.NAME);
        double angled_ty = LimelightHelpers.getTY(Constants.Limelight.NAME);
        double ty = Constants.Limelight.CAMERA_ANGLE + angled_ty;

        double dx = Constants.Limelight.TARGET_DISTANCE_AWAY - (Constants.Dimensions.APRILTAG_HEIGHT - Constants.Dimensions.CAMERA_HEIGHT) / Math.tan(Math.toRadians(ty));
        double dy = dx * Math.tan(Math.toRadians(tx));

        return List.of(
                new Pose2d(0, 0, s_Swerve.getGyroYaw()),
                new Pose2d(dx, dy, new Rotation2d(0)));
    }

    private List<Pose2d> calculateNotePoints() {
        double tx = LimelightHelpers.getTX(Constants.Limelight.NAME);
        double angled_ty = LimelightHelpers.getTY(Constants.Limelight.NAME);
        double ty = Constants.Limelight.CAMERA_ANGLE + angled_ty;

        double dx = (Constants.Dimensions.NOTE_HEIGHT - Constants.Dimensions.CAMERA_HEIGHT) / Math.tan(Math.toRadians(ty));
        double dy = dx * Math.tan(Math.toRadians(tx));

        return List.of(
                new Pose2d(0, 0, new Rotation2d(0)),
                new Pose2d(dx, dy, Rotation2d.fromDegrees(tx)));
    }
}