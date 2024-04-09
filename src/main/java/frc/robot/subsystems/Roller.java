package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Roller extends SubsystemBase {

    private TalonFX fxRollerMotor;
    private TalonFXConfiguration fxConfig;

    /**
     * Intake consists of 1 falcon500 motor to move the
     * conveyer belt and the intake wheels.
     */
    public Roller() {
        fxRollerMotor = new TalonFX(Constants.Roller.RollerMotor.MOTOR_ID);
        fxConfig = new TalonFXConfiguration();
        fxRollerMotor.getConfigurator().apply(fxConfig);

        fxRollerMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setSpeed(double speedPercent) {
        fxRollerMotor.set(speedPercent);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Roller Motor Angle", fxRollerMotor.getPosition().getValue());
    }
}
