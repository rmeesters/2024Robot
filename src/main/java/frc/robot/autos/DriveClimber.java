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
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.PneumaticsHandler;
import frc.robot.subsystems.Shooter;

public class DriveClimber extends SequentialCommandGroup {

        private final Climber s_Climber = RobotContainer.s_Climber;
        private final PneumaticsHandler s_PneumaticsHandler = RobotContainer.h_pneumatics;


    /**
     * This auto is for the robot to run in the autonomous state of
     * the round when in the middle position of the field.
     */
    public DriveClimber(double speed) {
        addCommands(
        new InstantCommand(()-> s_PneumaticsHandler.setClimber(false)),
        new WaitCommand(0.25),
        new InstantCommand(()-> s_Climber.setSpeed(speed))
        );
    }
}
