package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class AngleShooterCommand extends Command {

    private final Shooter s_Shooter = RobotContainer.s_Shooter;

    private double speed;

    public AngleShooterCommand(double speed) {
        this.speed = speed;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {

        boolean close = false;

        // Min and max
        if (speed < 0) {
            double distance = s_Shooter.getCanCoderPosition() - Constants.Shooter.CANCODER_MIN;

            if (distance < 0) {
                cancel();
                return;
            }
            
            if (distance < 0.5) {
                close = true;
            }
        }

        else if (speed > 0) {
            double distance = Constants.Shooter.CANCODER_MAX - s_Shooter.getCanCoderPosition();

            if (distance < 0) {
                cancel();
                return;
            }

            if (distance < 0.5) {
                close = true;
            }
        }

        s_Shooter.setShaftSpeed(close ? speed / 5 : speed);
    }

    @Override
    public void end(boolean interrupted) {
        s_Shooter.setShaftSpeed(0);
    }
    
}
