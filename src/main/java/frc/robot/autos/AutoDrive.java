// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDrive extends SequentialCommandGroup {

  private final Swerve s_Swerve = RobotContainer.s_Swerve;
  private final TrajectoryConfig config;
  private final Trajectory trajectory;
  private final ProfiledPIDController thetaController;

  public AutoDrive(List<Pose2d> points, boolean reversed) {
    config = new TrajectoryConfig(
        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

    thetaController = new ProfiledPIDController(
        Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    trajectory = TrajectoryGenerator.generateTrajectory(points, config);
    postTrajectoryToDashBoard();
    
    // Reset odometry and follow trajectory
    addCommands(
        new InstantCommand(() -> s_Swerve.setPose(trajectory.getInitialPose())),
        generateSwerveControllerCommand());
  }

  private SwerveControllerCommand generateSwerveControllerCommand() {
    return new SwerveControllerCommand(
        trajectory,
        s_Swerve::getPose,
        Constants.Swerve.swerveKinematics,
        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        thetaController,
        s_Swerve::setModuleStates,
        s_Swerve);
  }

  private void postTrajectoryToDashBoard() {
    SmartDashboard.putNumber("auto initial pose x", trajectory.getInitialPose().getX());
    SmartDashboard.putNumber("auto initial pose y", trajectory.getInitialPose().getY());
    SmartDashboard.putString("auto trajectory", trajectory.toString());
  }
}
