package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.autos.depricatedAutoDrive;

public class DriveToSpeaker extends Command {

    private boolean TAG_IS_VISIBLE;

    private depricatedAutoDrive auto;

    /**
     * Drive to speaker using limelight
     */
    public DriveToSpeaker() {

    }

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled.
     */
    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex(
                Constants.Limelight.Back.NAME,
                Constants.Limelight.Pipelines.SPEAKER);

        // Stop if target is not visible
        TAG_IS_VISIBLE = LimelightHelpers.getTV(Constants.Limelight.Back.NAME);
        if (!TAG_IS_VISIBLE) {
            System.out.println("No target visible");
            cancel();
            return;
        }

        List<Pose2d> points = calculateAprilTagPoints();
        System.out.println("Destination: " + points.get(1));
        auto = new depricatedAutoDrive(points, true);
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
        if (!TAG_IS_VISIBLE)
            return;

        auto.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return !TAG_IS_VISIBLE || auto.isFinished();
    }

    private List<Pose2d> calculateAprilTagPoints() {
        double TX = Math.toRadians(
                LimelightHelpers.getTX(Constants.Limelight.Back.NAME));
        double TY = Math.toRadians(
                Constants.Limelight.Back.ANGLE + LimelightHelpers.getTY(Constants.Limelight.Back.NAME));

        double distance = (Constants.Limelight.Back.HEIGHT - Constants.Map.Speaker.APRILTAG_HEIGHT) / Math.tan(TY) - 2;

        double dx = distance * Math.cos(TX);
        double dy = distance * Math.sin(TX);

        return List.of(
                new Pose2d(0, 0, new Rotation2d(0)),
                new Pose2d(dx, dy, new Rotation2d(TX)));
    }
}
