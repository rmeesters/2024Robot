package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    PIDController pid;

    /**
     * Drive to target using limelight
     * 
     * @param pipeline What preset values are wanted
     *                 (Constants.Limelight.Pipelines.<>)
     */
    public TargetSpeakerCommand() {
        pid = new PIDController(.08, 0, 0);
        // addRequirements(s_Shooter);
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

        // Rotate to speaker
        double translationVal = MathUtil.applyDeadband(
                -RobotContainer.driver.getRawAxis(RobotContainer.translationAxis), Constants.STICK_DEAD_BAND);
        double strafeVal = MathUtil.applyDeadband(-RobotContainer.driver.getRawAxis(RobotContainer.strafeAxis),
                Constants.STICK_DEAD_BAND);
        // double rotationVal = Math.cbrt(1.0 / Constants.Autos.LARGEST_POSSIBLE_ROTATION * getAngleDifference());
        double rotationVal = pid.calculate(LimelightHelpers.getTX(Constants.Limelight.Back.NAME), 0);
        s_Swerve.drive(new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_ANGULAR_VELOCITY),
                rotationVal, true, false);

        s_Shooter.setAngle(targetAngle);
        SmartDashboard.putNumber("target angle", targetAngle);
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

        double dy = -(Constants.Map.Speaker.APRILTAG_HEIGHT - Constants.Limelight.Back.HEIGHT);
        double dx = dy / Math.tan(TY);

        targetAngle = Math.atan((dy + Constants.Autos.SPEAKER_TO_APRILTAG_IN_METERS)
                / (dx + Constants.Autos.BACK_LIMELIGHT_TO_SHOOTER_PIVOT_IN_METERS));

        /*
        targetAngle = TY;
        // double aprilHeight = 60;
        // double c = Math.sqrt((21.5*21.5) + Math.pow((Math.sin(TY))/aprilHeight,2) -
        // 2*aprilHeight*(Math.sin(TY)/aprilHeight)*Math.cos(180-TY));
        // double b = Math.asin(Math.sin(180-TY)/c)*(Math.sin(TY)/aprilHeight);

        SmartDashboard.putNumber("dx", dx);
        SmartDashboard.putNumber("dy", dy);
        // SmartDashboard.putNumber("target angle", b);
        */
    }
}
