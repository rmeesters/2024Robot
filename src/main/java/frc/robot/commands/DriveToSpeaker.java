package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.autos.AutoDrive;
import frc.robot.subsystems.Swerve;

public class DriveToSpeaker extends Command {

    private final Swerve s_Swerve = RobotContainer.s_Swerve;

    private boolean TAG_IS_VISIBLE;

    private AutoDrive auto;

    /**
     * Drive to target using limelight
     * 
     * @param pipeline What preset values are wanted
     *                 (Constants.Limelight.Pipelines.<>)
     */
    public DriveToSpeaker() {

    }

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled.
     */
    @Override
    public void initialize() {
        int pipeline = RobotContainer.team == RobotContainer.RED ? Constants.Limelight.Pipelines.Speaker.Red.CENTER
                : Constants.Limelight.Pipelines.Speaker.Blue.CENTER;
        LimelightHelpers.setPipelineIndex(Constants.Limelight.Back.NAME, pipeline);

        // Stop if target is not visible
        TAG_IS_VISIBLE = LimelightHelpers.getTV(Constants.Limelight.Back.NAME);
        if (!TAG_IS_VISIBLE) {
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
        if (!TAG_IS_VISIBLE) {
            return;
        }

        auto.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return !TAG_IS_VISIBLE || auto.isFinished();
    }

    private List<Pose2d> calculateAprilTagPoints() {
        double TX = Math.toRadians(
                s_Swerve.getGyroYaw().getDegrees() + LimelightHelpers.getTX(Constants.Limelight.Back.NAME));
        double TY = Math.toRadians(
                Constants.Limelight.Back.CAMERA_ANGLE + LimelightHelpers.getTY(Constants.Limelight.Back.NAME));

        double distance = -(Constants.Limelight.Pipelines.Speaker.APRILTAG_HEIGHT
                - Constants.Limelight.Back.CAMERA_HEIGHT) / Math.tan(TY) - 2;

        double dx = distance * Math.cos(TX);
        double dy = distance * Math.sin(TX);

        return List.of(
                new Pose2d(0, 0, s_Swerve.getGyroYaw()),
                new Pose2d(dx, dy, new Rotation2d(0)));
    }
}
