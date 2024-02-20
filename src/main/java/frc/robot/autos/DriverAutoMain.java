package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveToNote;
import frc.robot.commands.DriveToSpeaker;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.MoveCommand;
import frc.robot.commands.ShootNote;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class DriverAutoMain extends SequentialCommandGroup {

    private final Shooter s_Shooter = RobotContainer.s_Shooter;
    private final Intake s_Intake = RobotContainer.s_Intake;
    private final Swerve s_Swerve = RobotContainer.s_Swerve;

    /**
     * This auto is for the robot to run in the autonomous state of
     * the round when in the middle position of the field.
     * 
     * <hr>
     * <h3>Instructions:</h3>
     * <ol>
     * <li>Zero shooter angle
     * <li>Shoot note into speaker
     * <li>Drive to middle note
     * <li>Pick up middle note
     * <li>Drive in range of speaker
     * <li>Shooter note into speaker
     * <li>Drive to left note
     * <li>Pick up left note
     * <li>Drive to speaker
     * <li>Shooter note into speaker
     */
    public DriverAutoMain() {
        addCommands(
                new InstantCommand(() -> s_Shooter.setShaftRotation(0)),
                new InstantCommand(() -> s_Shooter.setSpeed(1)),
                new InstantCommand(() -> s_Intake.setSpeed(1)),
                new WaitCommand(0.5),

                // Middle note
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                            new MoveCommand(List.of(
                                    new Pose2d(0, 0, new Rotation2d(0)),
                                    new Pose2d(1.3, 0, new Rotation2d(0))), false),
                            new MoveCommand(List.of(
                                    new Pose2d(0, 0, new Rotation2d(0)),
                                    new Pose2d(0.5, 0, new Rotation2d(0))), false)),
                        //new IntakeNote(),
                        new InstantCommand(() -> s_Shooter.setShaftRotation(10.7))),
                new WaitCommand(1),
                

                // Left note
                // new AutoDrive(List.of(
                //         new Pose2d(1.8, 0, new Rotation2d(0)),
                //         new Pose2d(1.0, 0.3, new Rotation2d(-45)),
                //         new Pose2d(1.8, 1, new Rotation2d(-45))), false),

                new ParallelCommandGroup(
                    new MoveCommand(List.of(
                            new Pose2d(1.8, 0, new Rotation2d(0)),
                            new Pose2d(1.6, 1, new Rotation2d(90)),
                            new Pose2d(1.6, 2, new Rotation2d(90))), false),
                    new IntakeNote()),
                    new MoveCommand(List.of(
                            new Pose2d(1.6, 2, new Rotation2d(0)),
                            new Pose2d(1.1, 1.5, new Rotation2d(-45))), false),
                
                // new ParallelCommandGroup(
                //         new DriveToNote(),
                //         new IntakeNote()),
                // new ParallelCommandGroup(
                //     new DriveToSpeaker(),
                //     new InstantCommand(() -> s_Shooter.angleToSpeaker())
                // ),
                // new ShootNote()
                new InstantCommand(() -> s_Shooter.setSpeed(0)),
                new InstantCommand(() -> s_Intake.setSpeed(0))
            );
    }
}
