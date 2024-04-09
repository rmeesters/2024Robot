package frc.robot.commands.shooter;

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
        if (speed == 0)
            cancel();
    }

    @Override
    public void execute() {
        double distance;

        // Min and max
        if (speed < 0)
            distance = s_Shooter.getCanCoderPosition() - Constants.Shooter.CANCODER_MIN;
        else
            distance = Constants.Shooter.CANCODER_MAX - s_Shooter.getCanCoderPosition();

        if (distance < 0) {
            cancel();
            return;
        }

        s_Shooter.setShaftSpeed(distance < 0.5 ? speed / 5 : speed);
    }

    @Override
    public void end(boolean interrupted) {
        s_Shooter.setShaftSpeed(0);
    }

}
