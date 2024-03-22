package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.PneumaticsHandler;

public class MoveClimberCommand extends Command {

    private final Climber s_Climber = RobotContainer.s_Climber;
    private final PneumaticsHandler h_pneumatics = RobotContainer.h_pneumatics;

    private final double speed;
    private final Timer timer = new Timer();

    public MoveClimberCommand(double speed) {
        this.speed = speed;
    }

    @Override
    public void initialize() {
        // Unlock climber
        h_pneumatics.setClimberSolenoid(false);
    }

    @Override
    public void execute() {
        // Start climber
        if (timer.hasElapsed(Constants.Climber.DELAY_AFTER_PNEUMATICS))
            s_Climber.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop climber
        s_Climber.setSpeed(0);

        // Lock climber
        new InstantCommand(() -> h_pneumatics.setClimberSolenoid(true));
    }
    
}
