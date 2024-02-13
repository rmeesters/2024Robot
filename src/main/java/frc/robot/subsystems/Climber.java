package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    public FalconMotor mClimberMotor = new FalconMotor(
            Constants.Climber.Motor.driveMotorID,
            -1, // No canCoder
            Constants.Climber.Motor.angleOffset);

    /**
     * Climber consists of 1 falcon500 motor to move the arms up and down.
     */
    public Climber() {

    }
    
}
