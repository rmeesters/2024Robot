package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.RobotContainer;
import frc.robot.commands.MoveCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PneumaticsHandler;
import frc.robot.subsystems.Shooter;

public class DriverAutoWallSide extends SequentialCommandGroup {

    private final Shooter s_Shooter = RobotContainer.s_Shooter;
    private final Intake s_Intake = RobotContainer.s_Intake;
    private final PneumaticsHandler h_pneumatics = RobotContainer.h_pneumatics;

    /**
     * This auto is for the robot to run in the autonomous state of
     * the round when in the middle position of the field.
     */
    public DriverAutoWallSide(boolean redTeam) {
        addCommands(
                // Loaded note
                new InstantCommand(() -> s_Shooter.setShaftRotation(0)),
                new InstantCommand(() -> s_Shooter.setSpeed(1)),
                new WaitCommand(0.5),
                new InstantCommand(() -> s_Intake.setSpeed(1)),
                new InstantCommand(() -> h_pneumatics.setShooterSolenoid(true)),
                new WaitCommand(1),

                // Middle note
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new MoveCommand(List.of(
                                        new Pose2d(0, 0, new Rotation2d(0)),
                                        new Pose2d(2, -0.75, new Rotation2d(0))), false),
                                new MoveCommand(List.of(
                                        new Pose2d(0, 0, new Rotation2d(0)),
                                        new Pose2d(0.5, 0, new Rotation2d(0))), false)),
                        new InstantCommand(() -> s_Shooter.setShaftRotation(10.7))),
                new WaitCommand(1),
        
                // Disable auto
                new InstantCommand(() -> h_pneumatics.setShooterSolenoid(false)),
                new InstantCommand(() -> s_Shooter.setSpeed(0)),
                new InstantCommand(() -> s_Intake.setSpeed(0)));
    }
}
