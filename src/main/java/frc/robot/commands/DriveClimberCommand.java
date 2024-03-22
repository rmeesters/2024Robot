package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.RobotContainer;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.PneumaticsHandler;

public class DriveClimberCommand extends SequentialCommandGroup {

    private final Climber s_Climber = RobotContainer.s_Climber;
    private final PneumaticsHandler s_PneumaticsHandler = RobotContainer.h_pneumatics;

    public DriveClimberCommand(double speed) {
        addCommands(
            new InstantCommand(()-> s_PneumaticsHandler.setClimberSolenoid(false)),
            new WaitCommand(0.25),
            new InstantCommand(()-> s_Climber.setSpeed(speed))
        );
    }
}
