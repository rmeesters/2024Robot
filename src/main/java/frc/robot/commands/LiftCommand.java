package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.PneumaticsHandler;

public class LiftCommand extends Command {

    private final Climber s_Climber = RobotContainer.s_Climber;

    private final PneumaticsHandler h_pneumatics = RobotContainer.h_pneumatics;

    private final double speed;

    private final Timer timer = new Timer();

    public LiftCommand(double speed) {
        this.speed = speed;
    }

    @Override
    public void initialize() {
        h_pneumatics.setClimber(false);
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(0.2))
            s_Climber.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        s_Climber.setSpeed(0);

        new SequentialCommandGroup(
            new WaitCommand(0.2),
            new InstantCommand(() -> h_pneumatics.setClimber(true))
        ).schedule();
    }
    
}
