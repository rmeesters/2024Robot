package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    private TalonFX fxClimberMotor;
    private TalonFXConfiguration fxConfig;

    /**
     * Intake consists of 1 falcon500 motor to move the
     * conveyer belt and the intake wheels.
     */
    public Climber() {
        fxClimberMotor = new TalonFX(Constants.Climber.Motor.driveMotorID);
        fxConfig = new TalonFXConfiguration();
        fxClimberMotor.getConfigurator().apply(fxConfig);
    }

    public void setSpeed(double speed) {
        fxClimberMotor.set(speed / Constants.Climber.Motor.maxSpeed);
    }

    //TODO Use MotionMagic

}
