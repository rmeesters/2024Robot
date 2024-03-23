package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
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

    public AmpCommand(boolean reversed) {
        b_reversed = reversed;
    }

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled.
     */
    @Override
    public void initialize() {
        h_pneumatics.setTiltSolenoid(true);

        if (b_reversed) return;

        s_Shooter.setShaftRotation(-29.76);
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        s_Roller.setSpeed(b_reversed ? 0.2 : -0.2);

        if (b_reversed) return;

        s_Shooter.setSpeed(0.2);
        s_Intake.setSpeed(0.2);
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
        h_pneumatics.setTiltSolenoid(false);
    }

    @Override
    public boolean isFinished() {
       return false;
    }
}
