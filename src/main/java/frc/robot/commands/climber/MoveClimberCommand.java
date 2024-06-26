package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.PneumaticsHandler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

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
        timer.restart();
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
        h_pneumatics.setClimberSolenoid(true);
        timer.stop();
    }

}
