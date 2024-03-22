package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.RobotContainer;
import frc.robot.commands.MoveCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PneumaticsHandler;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class DriverAutoMoveBack extends SequentialCommandGroup {

    private final Shooter s_Shooter = RobotContainer.s_Shooter;
    private final Swerve s_Swerve = RobotContainer.s_Swerve;
    private final Intake s_Intake = RobotContainer.s_Intake;
    private final PneumaticsHandler h_pneumatics = RobotContainer.h_pneumatics;

    /**
     * This auto is for the robot to run in the autonomous state of
     * the round when in the middle position of the field.
     */
    public DriverAutoMoveBack() {
        addCommands(
                // Loaded note
                new InstantCommand(() -> RobotContainer.gyro_temp = s_Swerve.getGyroYaw()),
                new InstantCommand(() -> s_Shooter.setSpeed(1)),
                // new InstantCommand(() -> s_Shooter.setShaftRotation(6.52)),
                new WaitCommand(0.5),
                new InstantCommand(() -> s_Intake.setSpeed(1)),
                new InstantCommand(() -> h_pneumatics.setShooterSolenoid(true)),
                new WaitCommand(1),
                new InstantCommand(() -> s_Shooter.setShaftRotation(11)),
                new MoveCommand(List.of(
                                        new Pose2d(0, 0, new Rotation2d(0)),
                                        new Pose2d(2, 0, new Rotation2d(0))), false),
                new MoveCommand(List.of(
                                        new Pose2d(0, 0, new Rotation2d(0)),
                                        new Pose2d(0.5, 0, new Rotation2d(0))), false),
                // Disable auto
                new InstantCommand(() -> s_Swerve.setHeading(RobotContainer.gyro_temp)),
                new InstantCommand(() -> h_pneumatics.setShooterSolenoid(false)),
                new InstantCommand(() -> s_Shooter.setSpeed(0)),
                new InstantCommand(() -> s_Shooter.setShaftRotation(0)),
                new InstantCommand(() -> s_Intake.setSpeed(0)));
    }
}
