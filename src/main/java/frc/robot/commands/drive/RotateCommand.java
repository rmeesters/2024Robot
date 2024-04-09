package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

public class RotateCommand extends Command {

    private final Swerve s_Swerve = RobotContainer.s_Swerve;

    private double targetRotation;

    public RotateCommand(double rotation) {
        targetRotation = rotation % 360;
    }

    @Override
    public void execute() {
        double translationVal = MathUtil.applyDeadband(
                -RobotContainer.driver.getRawAxis(RobotContainer.translationAxis), Constants.STICK_DEAD_BAND);
        double strafeVal = MathUtil.applyDeadband(
                -RobotContainer.driver.getRawAxis(RobotContainer.strafeAxis), Constants.STICK_DEAD_BAND);
        double rotationVal = Math.cbrt(1.0 / Constants.Autos.LARGEST_POSSIBLE_ROTATION * getAngleDifference());

        s_Swerve.drive(new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED), rotationVal, true,
                false);
    }

    public double getAngleDifference() {
        return s_Swerve.gyro.getYaw() - targetRotation;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(s_Swerve.gyro.getYaw() - targetRotation) < 1;
    }

}
