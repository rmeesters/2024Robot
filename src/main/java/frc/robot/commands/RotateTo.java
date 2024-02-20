package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

public class RotateTo extends Command {

    private final Swerve s_Swerve = RobotContainer.s_Swerve;

    private double targetRotation;

    public RotateTo() {
        targetRotation = 0;
    }

    public void setTarget(double rotation) {
        targetRotation = rotation%360;
    }

    @Override
    public void execute() {
        double translationVal = MathUtil.applyDeadband(-RobotContainer.driver.getRawAxis(RobotContainer.translationAxis), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(-RobotContainer.driver.getRawAxis(RobotContainer.strafeAxis), Constants.stickDeadband);
        s_Swerve.drive(new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), direction() / 8, true, false);
    }

    public double direction() {
        return s_Swerve.gyro.getYaw() - targetRotation;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(s_Swerve.gyro.getYaw() - targetRotation) < 1;
    }
    
}
