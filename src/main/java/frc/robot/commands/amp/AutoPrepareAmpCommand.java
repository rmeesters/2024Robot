package frc.robot.commands.amp;

import edu.wpi.first.wpilibj.Timer;
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

    private final Timer timer = new Timer();

    private final double STOP_DELAY = 0.9;

    public AutoPrepareAmpCommand() {

    }

    @Override
    public void initialize() {
        s_Shooter.setShaftRotation(Constants.Shooter.VERTICAL_POSITION);

        timer.restart();
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(2)) {
            h_pneumatics.setShooterSolenoid(true);
            s_Shooter.setSpeed(0.3);
            s_Intake.setSpeed(0.3);
            s_Roller.setSpeed(-0.5);

            return;
        }

        s_Intake.setSpeed(0.1);
        h_pneumatics.setTiltSolenoid(true);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            s_Shooter.setShaftRotation(Constants.Shooter.MOVE_POSITION);
        }

        h_pneumatics.setShooterSolenoid(false);
        s_Shooter.setSpeed(0);
        s_Intake.setSpeed(0);
        s_Roller.setSpeed(0);

        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2 + STOP_DELAY);
    }

}
