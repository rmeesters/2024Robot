package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Roller extends SubsystemBase {
    

    private TalonFX fxClimberMotor;
    private TalonFXConfiguration fxConfig;

    /**
     * Intake consists of 1 falcon500 motor to move the
     * conveyer belt and the intake wheels.
     */
    public Roller() {
        fxClimberMotor = new TalonFX(Constants.Roller.RollerMotor.MOTOR_ID);
        fxConfig = new TalonFXConfiguration();
        fxClimberMotor.getConfigurator().apply(fxConfig);

        //sfxClimberMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setSpeed(double speedPercent) {
        fxClimberMotor.set(speedPercent);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Roller Motor Angle", fxClimberMotor.getPosition().getValue());
    }
}
