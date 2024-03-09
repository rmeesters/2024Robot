package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.RobotContainer;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveCommand;
import frc.robot.commands.RotateCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PneumaticsHandler;
import frc.robot.subsystems.Shooter;

public class SpitAndMove extends SequentialCommandGroup {

    private final Shooter s_Shooter = RobotContainer.s_Shooter;
    private final Intake s_Intake = RobotContainer.s_Intake;
    private final PneumaticsHandler h_pneumatics = RobotContainer.h_pneumatics;

    /**
     * This auto is for the robot to run in the autonomous state of
     * the round when in the middle position of the field.
     */
    public SpitAndMove() {
        addCommands(
                // Loaded note
                new WaitCommand(3),
                new InstantCommand(() -> s_Intake.setSpeed(-0.6)),
                new WaitCommand(1.5),
                new MoveCommand(List.of(
                                        new Pose2d(0, 0, new Rotation2d(0)),
                                        new Pose2d(-3, -2, new Rotation2d(0))), true),
                // Disable auto
                new InstantCommand(() -> s_Intake.setSpeed(0)));
    }
}
