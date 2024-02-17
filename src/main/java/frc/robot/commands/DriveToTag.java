package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.autos.AutoDrive;
import frc.robot.subsystems.Swerve;

public class DriveToTag extends Command {

    private final Swerve s_Swerve = RobotContainer.s_Swerve;

    private final int PIPELINE;

    private boolean TARGET_IS_VISIBLE;

    private AutoDrive auto;

    /**
     * Drive to target using limelight
     * 
     * @param pipeline What preset values are wanted
     *                 (Constants.Limelight.Pipelines.<>)
     */
    public DriveToTag(int pipeline) {
        PIPELINE = pipeline;
    }

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled.
     */
    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex(Constants.Limelight.Back.NAME, PIPELINE);

        // Stop if target is not visible
        TARGET_IS_VISIBLE = LimelightHelpers.getTV(Constants.Limelight.Back.NAME);
        if (!TARGET_IS_VISIBLE) {
            System.out.println("No target visible");
            cancel();
            return;
        }

        List<Pose2d> points = calculateAprilTagPoints();
        auto = new AutoDrive(points, true);
        auto.initialize();
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
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
            return;
        }

        auto.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return !TARGET_IS_VISIBLE || auto.isFinished();
    }

    private List<Pose2d> calculateAprilTagPoints() {
        double tx = LimelightHelpers.getTX(Constants.Limelight.Back.NAME);
        double angled_ty = LimelightHelpers.getTY(Constants.Limelight.Back.NAME);
        double ty = Constants.Limelight.Back.CAMERA_ANGLE + angled_ty;

        double dx = Constants.Limelight.TARGET_TAG_DISTANCE
                - (Constants.Dimensions.APRILTAG_HEIGHT - Constants.Limelight.Back.CAMERA_HEIGHT)
                        / Math.tan(Math.toRadians(ty));
        double dy = dx * Math.tan(Math.toRadians(tx));

        return List.of(
                new Pose2d(0, 0, s_Swerve.getGyroYaw()),
                new Pose2d(dx, dy, new Rotation2d(0)));
    }
}
