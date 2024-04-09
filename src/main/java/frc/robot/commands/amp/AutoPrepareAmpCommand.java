package frc.robot.commands.amp;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.PneumaticsHandler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.Shooter;

public class AutoPrepareAmpCommand extends Command {

    private final Shooter s_Shooter = RobotContainer.s_Shooter;
    private final Roller s_Roller = RobotContainer.s_Roller;
    private final Intake s_Intake = RobotContainer.s_Intake;
    private final PneumaticsHandler h_pneumatics = RobotContainer.h_pneumatics;

    private static Command command;
    
    public AutoPrepareAmpCommand() {
        command = new SequentialCommandGroup(
                // Angle and Prepare
                new InstantCommand(() -> {
                    s_Shooter.setShaftRotation(Constants.Shooter.VERTICAL_POSITION);
                    s_Intake.setSpeed(0.1);
                    h_pneumatics.setTiltSolenoid(true);
                }),
                new WaitCommand(2),
                //  Load Rollers
                new InstantCommand(() -> {
                    h_pneumatics.setShooterSolenoid(true);
                    s_Shooter.setSpeed(0.3);
                    s_Intake.setSpeed(0.3);
                    s_Roller.setSpeed(-0.5);
                }),
                new WaitCommand(0.9),
                //  Stop
                new InstantCommand(() -> {
                    s_Shooter.setSpeed(0);
                    s_Intake.setSpeed(0);
                    s_Roller.setSpeed(0);
                    h_pneumatics.setShooterSolenoid(false);
                }));
    }

    @Override
    public void initialize() {
        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();
    }


    @Override
    public void end(boolean interrupted) {
        command.cancel();

        s_Shooter.setShaftRotation(Constants.Shooter.MOVE_POSITION);
        h_pneumatics.setShooterSolenoid(false);
        s_Shooter.setSpeed(0);
        s_Intake.setSpeed(0);
        s_Roller.setSpeed(0);
    }

}
