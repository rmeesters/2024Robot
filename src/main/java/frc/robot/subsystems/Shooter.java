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
        fxLeftMotor = new TalonFX(Constants.Shooter.WheelMotor.LEFT_MOTOR_ID);
        fxRightMotor = new TalonFX(Constants.Shooter.WheelMotor.RIGHT_MOTOR_ID);
        fxAngleMotor = new TalonFX(Constants.Shooter.AngleMotor.MOTOR_ID);

        // Configure motors
        fxShooterConfig = new TalonFXConfiguration();
        fxAngleConfig = new TalonFXConfiguration();

        // Make shaft listen to canCoder
        fxAngleConfig.Feedback.FeedbackRemoteSensorID = Constants.Shooter.AngleMotor.CANCODER_ID;
        fxAngleConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        // Acceleration and decceleration of shaft
        MotionMagicConfigs angleMotionMagic = fxAngleConfig.MotionMagic;
        angleMotionMagic.MotionMagicAcceleration = Constants.Shooter.AngleMotor.ACCELERATION;
        angleMotionMagic.MotionMagicCruiseVelocity = Constants.Shooter.AngleMotor.MAX_SPEED;

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
        fxRightMotor.setNeutralMode(NeutralModeValue.Coast);
        fxLeftMotor.setNeutralMode(NeutralModeValue.Coast);

        // Define canCoder for printing angle to the dashboard
        angleCanCoder = new CANcoder(Constants.Shooter.AngleMotor.CANCODER_ID);
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
     * Set the position of the shaft.
     * 
     * @param rotation CanCoder Value
     * @return Time to complete
     */
    public void setShaftRotation(double rotation) {
        if (rotation > Constants.Shooter.CANCODER_MAX || rotation < Constants.Shooter.CANCODER_MIN) {
            System.err.println(rotation + " is not a valid shaft rotation (max: 11, min: -32)");
            return;
        }
        System.err.println("rotation:" + rotation);
        fxAngleMotor.setControl(m_mmReq.withPosition(rotation).withSlot(0));
    }

    /**
     * Set the position of the shaft based on angle assuming the
     * robot started with the ramp all the way down
     * 
     * @param degrees
     */
    public void setAngle(double degrees) {
        double targetEncoderValue = (55-degrees)*0.78;

        if (targetEncoderValue < Constants.Shooter.CANCODER_MIN)
            targetEncoderValue = Constants.Shooter.CANCODER_MIN;

        else if (targetEncoderValue > Constants.Shooter.CANCODER_MAX)
            targetEncoderValue = Constants.Shooter.CANCODER_MAX;

        setShaftRotation(targetEncoderValue);
    }

    // public void angleToSpeaker() {
    //     double TY = Math.toRadians(
    //             Constants.Limelight.Back.ANGLE + LimelightHelpers.getTY(Constants.Limelight.Back.NAME));

    //     double dy = -(Constants.Map.Speaker.APRILTAG_HEIGHT - Constants.Limelight.Back.HEIGHT);
    //     double dx = dy / Math.tan(TY);

    //     double angle = Math.atan((dy + 0.6) / (dx + 0.5));
    //     setAngle(angle);
    // }

    /** Set position of shooter to initialized position (zero on cancoder) */
    public void setZero() {
        angleCanCoder.setPosition(0);
    }

    public double getRPS() {
        return fxLeftMotor.getVelocity().getValue();
    }

    public double getCanCoderPosition() {
        return angleCanCoder.getPosition().getValue();
    }

    /**
     * Update dashboard with rotation values
     */
    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Angle Motor Angle",
        // fxAngleMotor.getPosition().getValue());
        SmartDashboard.putNumber("shooter rps", getRPS());
        SmartDashboard.putNumber("Angle CanCoder Angle", angleCanCoder.getPosition().getValue());
        SmartDashboard.putNumber("Left Shooter Velocity", fxLeftMotor.getVelocity().getValue());
        SmartDashboard.putNumber("Right Shooter Velocity", fxRightMotor.getVelocity().getValue());
        SmartDashboard.putNumber("Right Shooter Velocity", fxAngleMotor.getVelocity().getValue());
    }

}
