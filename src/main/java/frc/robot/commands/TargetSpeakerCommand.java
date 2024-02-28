package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class TargetSpeakerCommand extends Command {

    private final Swerve s_Swerve = RobotContainer.s_Swerve;
    private final Shooter s_Shooter = RobotContainer.s_Shooter;

    private boolean TAG_IS_VISIBLE;

    private double targetRotation, targetAngle;

    /**
     * Drive to target using limelight
     * 
     * @param pipeline What preset values are wanted
     *                 (Constants.Limelight.Pipelines.<>)
     */
    public TargetSpeakerCommand() {

    }

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled.
     */
    @Override
    public void initialize() {
        int pipeline = Constants.Limelight.Pipelines.SPEAKER;
        LimelightHelpers.setPipelineIndex(Constants.Limelight.Back.NAME, pipeline);
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        calculateValues();

        double translationVal = MathUtil.applyDeadband(
                -RobotContainer.driver.getRawAxis(RobotContainer.translationAxis), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(-RobotContainer.driver.getRawAxis(RobotContainer.strafeAxis),
                Constants.stickDeadband);
        double rotationVal = Math.cbrt(1.0 / Constants.Autos.largestPossibleRotation * getAngleDifference());
        s_Swerve.drive(new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), rotationVal, true,
                false);

        s_Shooter.setAngle(targetAngle);
    }

    public double getAngleDifference() {
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

    }

    private void calculateValues() {
        TAG_IS_VISIBLE = LimelightHelpers.getTV(Constants.Limelight.Back.NAME);
        if (!TAG_IS_VISIBLE)
            return;

        targetRotation = s_Swerve.gyro.getYaw() + LimelightHelpers.getTX(Constants.Limelight.Back.NAME);
        double TY = Constants.Limelight.Back.ANGLE + LimelightHelpers.getTY(Constants.Limelight.Back.NAME);

        double dy = -(Constants.Map.Speaker.APRILTAG_HEIGHT
                - Constants.Limelight.Back.HEIGHT);
        double dx = dy / Math.tan(TY);

        targetAngle = Math.atan((dy + 0.6) / (dx + 0.56));
    }
}
