package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        fxClimberMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setSpeed(double speed) {
        double rotation = fxClimberMotor.getPosition().getValue();

        if (rotation > 100) {
            speed = Math.min(speed, 0);
        }

        if (rotation < 0) {
            speed = Math.max(speed, 0);
        }

        fxClimberMotor.set(speed / Constants.Climber.Motor.maxSpeed);
    }

    @Override
    public void periodic() {
        double rotation = fxClimberMotor.getPosition().getValue();

        SmartDashboard.putNumber("Climber Motor Angle", rotation);

        // if (rotation > 100 && fxClimberMotor.getVelocity().getValue() > 0) {

        // }
    }

    //TODO Use MotionMagic

}
