package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.autos.AutoDrive;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class FocusSpeaker extends Command {

    private final Swerve s_Swerve = RobotContainer.s_Swerve;
    private final Shooter s_Shooter = RobotContainer.s_Shooter;

    private boolean TAG_IS_VISIBLE;

    private double targetRotation, targetAngle;
    //angle;

    /**
     * Drive to target using limelight
     * 
     * @param pipeline What preset values are wanted
     *                 (Constants.Limelight.Pipelines.<>)
     */
    public FocusSpeaker() {

    }

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled.
     */
    @Override
    public void initialize() {
        int pipeline = Constants.Limelight.Pipelines.SPEAKER;
        LimelightHelpers.setPipelineIndex(Constants.Limelight.Back.NAME, pipeline);

        // Stop if target is not visible

        //angle = new RotateTo(targetAngle);

        //rotate.initialize();
        //angle.initialize();
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        calculateValues();

        double translationVal = MathUtil.applyDeadband(-RobotContainer.driver.getRawAxis(RobotContainer.translationAxis), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(-RobotContainer.driver.getRawAxis(RobotContainer.strafeAxis), Constants.stickDeadband);
        s_Swerve.drive(new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), direction() / 15, true, false);
        
        //angle.execute();
    }

    public double direction() {
        return atDesination() ? 0 : s_Swerve.gyro.getYaw() - targetRotation;
    }

    public boolean atDesination() {
        return Math.abs(s_Swerve.gyro.getYaw() - targetRotation) < 1;
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

        //angle.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return false; //rotate.isFinished();// && angle.isFinished();
    }

    private void calculateValues() {
        TAG_IS_VISIBLE = LimelightHelpers.getTV(Constants.Limelight.Back.NAME);
        if (!TAG_IS_VISIBLE) return;

        targetRotation = s_Swerve.gyro.getYaw() + LimelightHelpers.getTX(Constants.Limelight.Back.NAME);
        double TY = Constants.Limelight.Back.CAMERA_ANGLE + LimelightHelpers.getTY(Constants.Limelight.Back.NAME);

        double dy = -(Constants.Limelight.Pipelines.Speaker.APRILTAG_HEIGHT
                - Constants.Limelight.Back.CAMERA_HEIGHT);
        double dx = dy / Math.tan(TY);

        targetAngle = Math.atan((dy + 0.6) / (dx + 0.56));
    }
}
