package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
     * Shooter consists of 3 falcon500 motors, 2 to accellerate the projectile, and
     * 1 to angle the shooter.
     */
    public Shooter() {
        // Define motors
        fxLeftMotor = new TalonFX(Constants.Shooter.ShooterMotor.leftMotorID);
        fxRightMotor = new TalonFX(Constants.Shooter.ShooterMotor.rightMotorID);
        fxAngleMotor = new TalonFX(Constants.Shooter.AngleMotor.driveMotorID);

        // Configure motors
        fxShooterConfig = new TalonFXConfiguration();
        fxAngleConfig = new TalonFXConfiguration();

        // Make shaft listen to canCoder
        fxAngleConfig.Feedback.FeedbackRemoteSensorID = Constants.Shooter.AngleMotor.canCoderID;
        fxAngleConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        // Acceleration and decceleration of shaft
        MotionMagicConfigs angleMotionMagic = fxAngleConfig.MotionMagic;
        angleMotionMagic.MotionMagicAcceleration = Constants.Shooter.AngleMotor.shaftAcceleration;
        angleMotionMagic.MotionMagicCruiseVelocity = Constants.Shooter.AngleMotor.shaftMaxSpeed;

        Slot0Configs slot0 = fxAngleConfig.Slot0;
        slot0.kP = Constants.Shooter.AngleMotor.KP;
        slot0.kI = Constants.Shooter.AngleMotor.KI;
        slot0.kD = Constants.Shooter.AngleMotor.KD;

        // Apply configurations
        fxLeftMotor.getConfigurator().apply(fxShooterConfig);
        fxRightMotor.getConfigurator().apply(fxShooterConfig);
        fxAngleMotor.getConfigurator().apply(fxAngleConfig);

        // Make angle motor stop when not accelerating
        fxAngleMotor.setNeutralMode(NeutralModeValue.Brake);

        // Define canCoder for printing angle to the dashboard
        angleCanCoder = new CANcoder(Constants.Shooter.AngleMotor.canCoderID);
    }

    /**
     * Set the speed of the 2 talonfx shooter motors.
     * 
     * @param speedPercent Percent of max speed (-1 - 1)
     */
    public void setSpeed(double speedPercent) {
        fxLeftMotor.set(-speedPercent);
        fxRightMotor.set(speedPercent);
    }

    /**
     * Set the speed of the talonfx shaft motor.
     * 
     * @param speedPercent Percent of max speed (-1 - 1)
     */
    public void setShaftSpeed(double speedPercent) {
        fxAngleMotor.set(speedPercent);
    }

    /**
     * Set the position of the shaft assuming the robot started with 
     * the ramp all the way down.
     * 
     * @param rotation 0 - Constants.Shooter.canCoderLimit
     * @return Time to complete
     */
    public double setShaftRotation(double rotation) {
        fxAngleMotor.setControl(m_mmReq.withPosition(rotation).withSlot(0));

        //TODO Tune this
        double travelTime = 0.2 + Math.abs( angleCanCoder.getPosition().getValue() - rotation );
        return travelTime;
    }

    /**
     * Set the position of the shaft based on angle assuming the
     * robot started with the ramp all the way down
     * 
     * @param degrees
     */
    public double setAngle(double degrees) {
        // 0 is down, 1 is up
        double positionOnShaftPercentage = 1 - targetPositionInInches(degrees) / Constants.Shooter.ArmRange;

        // Range of 0 to cancoder limit
        double targetEncoderValue = Constants.Shooter.canCoderLimit * positionOnShaftPercentage;

        // Rotate sahft to calculated position
        SmartDashboard.putNumber("Target Encoder Value (Shaft)", targetEncoderValue);
        return setShaftRotation(targetEncoderValue);
    }

    /**
     * Set position of shaft based on inches from 
     * the canCoder zero (lowest angle of shooter).
     * 
     * @param distanceInInches Distance in inches
     */
    public void setShaftPosition(double distanceInInches) {
        // 0 is down, 1 is up
        double positionOnShaftPercentage = distanceInInches / Constants.Shooter.ArmRange;

        if (positionOnShaftPercentage > 1 || positionOnShaftPercentage < 0) {
            System.err.println("Invalid shaft position of " + distanceInInches + " inches");
        }

        // Range of 0 to cancoder limit
        double targetEncoderValue = Constants.Shooter.canCoderLimit * positionOnShaftPercentage;

        // Rotate sahft to calculated position
        SmartDashboard.putNumber("Target Encoder Value (Shaft)", targetEncoderValue);
        setShaftRotation(targetEncoderValue);
    }

    private double targetPositionInInches(double degrees) {
        // Do NOT change these constants
        final double A = -0.0190931;
        final double B = 0.13371;
        final double C = -5.7094;
        final double D = 91.7202;

        return A * Math.pow(degrees, 3)
                + B * Math.pow(degrees, 2)
                + C * degrees
                + D;
    }

    /**
     * Update dashboard with rotation values
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Angle Motor Angle", fxAngleMotor.getPosition().getValue());
        SmartDashboard.putNumber("Angle CanCoder Angle", angleCanCoder.getPosition().getValue());
    }

    

}
