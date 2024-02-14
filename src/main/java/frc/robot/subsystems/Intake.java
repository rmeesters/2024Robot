package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    
    public FalconMotor mConveyerMotor;

    /**
     * Intake consists of 1 falcon500 motor to move the conveyer belt and the intake wheels.
     */
    public Intake() {
        mConveyerMotor = new FalconMotor(Constants.Intake.Motor.driveMotorID);
    }

    public void setSpeed(double speed, boolean isOpenLoop) {
        mConveyerMotor.setSpeed(speed, isOpenLoop);
    }

    public void stopIntake() {
        mConveyerMotor.setSpeed(0, true);
    }
    
}
