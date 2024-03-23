package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PneumaticsHandler;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.Shooter;

public class AmpCommand extends Command {

    private final Intake s_Intake = RobotContainer.s_Intake;
    private final Roller s_Roller = RobotContainer.s_Roller;
    private final Shooter s_Shooter = RobotContainer.s_Shooter;

    private final PneumaticsHandler h_pneumatics = RobotContainer.h_pneumatics;
    
    private final boolean b_reversed;
    
    private final Timer timer = new Timer();

    private final double SHOOTER_SPEED = 0.1;
    private final double ROLLER_SPEED = -0.3;
    private final double ROLLER_REVERSED_SPEED = 0.6;
    private final double INTAKE_SPEED = 0.2;
    private final double ROLLER_DELAY = 0.2;

    public AmpCommand(boolean reversed) {
        b_reversed = reversed;
    }

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled.
     */
    @Override
    public void initialize() {
        timer.restart();
        
        h_pneumatics.setTiltSolenoid(true);

        if (b_reversed) return;

        h_pneumatics.setShooterSolenoid(true);
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        if (!b_reversed || timer.hasElapsed(ROLLER_DELAY))
            s_Roller.setSpeed(b_reversed ? ROLLER_REVERSED_SPEED : ROLLER_SPEED);

        if (b_reversed) return;

        s_Shooter.setSpeed(SHOOTER_SPEED);
        s_Intake.setSpeed(INTAKE_SPEED);
    }

    /**
     * The action to take when the command ends.
     * Called either when the command finishes normally or interrupted/canceled.
     *
     * Do not schedule commands here that share requirements with this command.
     * Use {@link #andThen(Command...)} instead.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        s_Roller.setSpeed(0);
        s_Intake.setSpeed(0);
        s_Shooter.setSpeed(0);
        h_pneumatics.setShooterSolenoid(false);
        h_pneumatics.setTiltSolenoid(false);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
       return false;
    }
}
