package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AngleShooter extends Command {

    private final Shooter s_Shooter = RobotContainer.s_Shooter;
    private final Intake s_Intake = RobotContainer.s_Intake;

    private double targetShaftPosition;
    private double targetAngle;

    private final Timer timer = new Timer();

    // Speed of angle shooter
    public static final double speed = 1.0;

    public AngleShooter(double degrees) {
        targetAngle = degrees;
    }

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled.
     */
    @Override
    public void initialize() {
        calculateTargetShaftPositionAt(targetAngle);

        // Start timer to create ordered sequences
        timer.restart();
    }

    /**
     * Get the distance the base of the arm must travel
     * 
     * @param degrees angle starting from the horizontal facing the back
     * @return The adjustment in inches
     */
    private void calculateTargetShaftPositionAt(double degrees) {
        final double A = -0.0156853;
        final double B = 0.166208;
        final double C = -6.04656;
        final double D = 97.6943;

        targetShaftPosition = A * Math.pow(degrees, 3) + B * Math.pow(degrees, 2) + C * degrees + D;
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        if (targetShaftPosition < s_Shooter.boltPosition)
            s_Shooter.setAngleSpeed(speed, false);
        else
            s_Shooter.setAngleSpeed(-speed, false);

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
        s_Shooter.boltPosition = targetShaftPosition;
        timer.stop();
        s_Shooter.stopAngleAdjustment();
        s_Intake.stopIntake();
        SmartDashboard.putBoolean("Angle Reached", true);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(Math.abs(targetShaftPosition - s_Shooter.boltPosition) / speed);
    }
}
