package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    
    private TalonFX fxIntakeMotor;
    private TalonFXConfiguration fxConfig;

    /**
     * Intake consists of 1 falcon500 motor to move the conveyer belt and the intake wheels.
     */
    public Intake() {
        fxIntakeMotor = new TalonFX(Constants.Intake.Motor.driveMotorID);
        fxConfig = new TalonFXConfiguration();
        fxIntakeMotor.getConfigurator().apply(fxConfig);
    }

    public void setSpeed(double speed) {
        fxIntakeMotor.set(speed / Constants.Intake.Motor.maxSpeed);
    }

    public void stopIntake() {
        setSpeed(0);
    }
    
}
