package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    
    private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
    
    private TalonFX fxLeftMotor;
    private TalonFX fxRightMotor;
    private TalonFXConfiguration fxShooterConfig;

    private TalonFX fxAngleMotor;
    private TalonFXConfiguration fxAngleConfig;

    private CANcoder angleCanCoder;

    /**
     * Shooter consists of 3 falcon500 motors, 2 to accellerate the projectile, and 1 to angle the shooter.
     */
    public Shooter() {
        fxLeftMotor = new TalonFX(Constants.Shooter.ShooterMotor.leftMotorID);
        fxRightMotor = new TalonFX(Constants.Shooter.ShooterMotor.rightMotorID);
        fxAngleMotor = new TalonFX(Constants.Shooter.AngleMotor.driveMotorID);
        
        fxShooterConfig = new TalonFXConfiguration();
        fxAngleConfig = new TalonFXConfiguration();

        MotionMagicConfigs angleMotionMagic = fxAngleConfig.MotionMagic;
        angleMotionMagic.MotionMagicCruiseVelocity = Constants.Shooter.AngleMotor.shaftAcceleration;
        angleMotionMagic.MotionMagicCruiseVelocity = Constants.Shooter.AngleMotor.shaftMaxSpeed;

        Slot0Configs slot0 = fxAngleConfig.Slot0;
        slot0.kP = Constants.Shooter.AngleMotor.KP;
        slot0.kI = Constants.Shooter.AngleMotor.KI;
        slot0.kD = Constants.Shooter.AngleMotor.KD;

        angleCanCoder = new CANcoder(Constants.Shooter.AngleMotor.canCoderID);
        
        fxLeftMotor.getConfigurator().apply(fxShooterConfig);
        fxRightMotor.getConfigurator().apply(fxShooterConfig);
        fxAngleMotor.getConfigurator().apply(fxAngleConfig);
    }

    public void setShooterSpeed(double speed) {
        fxLeftMotor.set(-speed / Constants.Shooter.ShooterMotor.maxSpeed);
        fxRightMotor.set(speed / Constants.Shooter.ShooterMotor.maxSpeed);
    }

    public void stopShooter() {
        setShooterSpeed(0);
    }

    public void setAngleAdjustmentSpeed(double speed) {
        fxAngleMotor.set(speed / Constants.Shooter.AngleMotor.shaftMaxSpeed);
    }

    public void setArmPosition(double position) {
        fxAngleMotor.setControl(m_mmReq.withPosition(position).withSlot(0));
    }

    public double getArmPosition() {
        return 0;
    }

    // public void setAngleAdjustmentSpeed(double speed) {
    //     fxAngleMotor.set(speed);
    // }

    // public void stopAngleAdjustment() {
    //     setAngleAdjustmentSpeed(0);
    // }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Angle Motor Angle", fxAngleMotor.getPosition().getValue());
        SmartDashboard.putNumber("Angle CanCoder Angle", angleCanCoder.getPosition().getValue());
    }

}
