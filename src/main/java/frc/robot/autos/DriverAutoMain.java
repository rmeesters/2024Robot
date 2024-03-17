package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.RobotContainer;
//import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveCommand;
import frc.robot.commands.RotateCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PneumaticsHandler;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class DriverAutoMain extends SequentialCommandGroup {

    private final Shooter s_Shooter = RobotContainer.s_Shooter;
    private final Swerve s_Swerve = RobotContainer.s_Swerve;
    private final Intake s_Intake = RobotContainer.s_Intake;
    private final PneumaticsHandler h_pneumatics = RobotContainer.h_pneumatics;

    /**
     * This auto is for the robot to run in the autonomous state of
     * the round when in the middle position of the field.
     */
    public DriverAutoMain(boolean redTeam) {
        addCommands(
                /* Loaded Note */

                // Angle to speaker and activate shooters
                new InstantCommand(() -> s_Shooter.setShaftRotation(0)),
                new InstantCommand(() -> s_Shooter.setSpeed(1)),
                new InstantCommand(() -> s_Intake.setSpeed(1)),
                new InstantCommand(() -> h_pneumatics.setShooterSolenoid(true)),
                new WaitCommand(0.5),

                /* Middle Note */

                new ParallelCommandGroup(
                        // Move to note
                        new SequentialCommandGroup(
                                new MoveCommand(List.of(
                                        new Pose2d(0, 0, new Rotation2d(0)),
                                        new Pose2d(1.3, 0, new Rotation2d(0))), false),
                                new MoveCommand(List.of(
                                        new Pose2d(0, 0, new Rotation2d(0)),
                                        new Pose2d(0.5, 0, new Rotation2d(0))), false)),
                        new InstantCommand(() -> s_Shooter.setShaftRotation(11))),
                new WaitCommand(1),

                // Left note
                //new InstantCommand(()-> s_Shooter.setSpeed(0)),
                new InstantCommand(() -> h_pneumatics.setShooterSolenoid(false)),
                new RotateCommand(redTeam ? 90 : -90), 
                new MoveCommand(List.of(
                        new Pose2d(0,0, new Rotation2d(0)),
                        new Pose2d(1.6, redTeam ? 0.1 : -0.1, new Rotation2d(0))), false),
                new InstantCommand(() -> s_Intake.setSpeed(0)),
                new RotateCommand(redTeam ? 30 : -30),

                // new MoveCommand(List.of(
                //         new Pose2d(0, 0, new Rotation2d(0)),
                //         new Pose2d(1, 0, new Rotation2d(0))),false),
                new InstantCommand(() -> s_Intake.setSpeed(1)),
                new InstantCommand(() -> h_pneumatics.setShooterSolenoid(true)),
                new WaitCommand(1),
                
                // Disable auto
                new InstantCommand(() -> s_Swerve.setHeading(Rotation2d.fromDegrees(redTeam ? -30 : 30))),
                new InstantCommand(() -> h_pneumatics.setShooterSolenoid(false)),
                new InstantCommand(() -> s_Shooter.setShaftRotation(0)),
                new InstantCommand(() -> s_Shooter.setSpeed(0)),
                new InstantCommand(() -> s_Intake.setSpeed(0)));
    }
}
