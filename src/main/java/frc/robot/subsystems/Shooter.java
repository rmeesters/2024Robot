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
     * @param rotation 0 - Constants.Shooter.canCoderLimit
     * @return Time to complete
     */
    public void setShaftRotation(double rotation) {
        /*if (rotation > Constants.Shooter.canCoderMax || rotation < Constants.Shooter.canCoderMin) {
            System.err.println(rotation + " is not a valid shaft rotation (max: 11, min: -32)");
            return;
        }*/
        System.err.println("rotation:" + rotation);
        fxAngleMotor.setControl(m_mmReq.withPosition(rotation).withSlot(0));
    }

    /**
     * Set position of shaft based on inches from
     * the canCoder zero (lowest angle of shooter).
     * 
     * @param distanceInInches Distance in inches
     */
    public void setShaftPosition(double distanceInInches) {
        // 0 is down, 1 is up
        double positionOnShaftPercentage = distanceInInches / Constants.Shooter.SHAFT_LENGTH_IN_INCHES;

        /*if (positionOnShaftPercentage > 1 || positionOnShaftPercentage < 0) {
            System.err.println("Invalid shaft position of " + distanceInInches + " inches");
            return;
        }*/

        // Range of 0 to cancoder limit
        double targetEncoderValue = (Constants.Shooter.CANCODER_MIN - Constants.Shooter.CANCODER_MAX)
                * positionOnShaftPercentage + Constants.Shooter.CANCODER_MAX;

        // Rotate sahft to calculated position
        setShaftRotation(targetEncoderValue);
    }

    /**
     * Set the position of the shaft based on angle assuming the
     * robot started with the ramp all the way down
     * 
     * @param degrees
     */
    public void setAngle(double degrees) {
        double targetEncoderValue = (55-degrees)*0.78;
        if (degrees < 41 || degrees > 90) {
            System.err.println(degrees + " is not a valid angle (max: 92, min: 45)");
        }
        if(degrees < 41) {
            targetEncoderValue = (55-41)*0.78;
        } else if (degrees > 90) {
            targetEncoderValue = (55-90)*0.78;
        }

        /*// 0 is down, 1 is up
        double positionOnShaftPercentage = 1 - calculatePositionInInches(degrees) / Constants.Shooter.shaftLength;

        // Range of 0 to cancoder limit
        double targetEncoderValue = (Constants.Shooter.CANCODER_MIN - Constants.Shooter.CANCODER_MAX)
                * positionOnShaftPercentage + Constants.Shooter.CANCODER_MAX;

        // Rotate sahft to calculated position
        SmartDashboard.putNumber("Target Angle", degrees);
        SmartDashboard.putNumber("Target Rotation", targetEncoderValue);*/
        setShaftRotation(targetEncoderValue);
    }

    /**
     * Get the position the shaft must be in to have a given angle.
     * 
     * @param degrees Angle from the horizontal
     * @return Inches from the highest angle
     */
    private double calculatePositionInInches(double degrees) {
        // Do NOT change these constants
        final double A = -0.0190931;
        final double B = 0.13371;
        final double C = -5.7094;
        final double D = 91.7202 - degrees;

        final double P = -B / A / 3;
        final double Q = Math.pow(P, 3) + (B * C - 3 * A * D) / 6 / Math.pow(A, 2);
        final double R = C / 3 / A;
        final double G = Math.sqrt(Math.pow(Q, 2) + Math.pow(R - Math.pow(P, 2), 3));

        return Math.cbrt(Q + G) + Math.cbrt(Q - G) + P + 0.044;
    }

    public void prepareIntake() {
        setShaftPosition(Constants.Shooter.IDEAL_INTAKE_POSITION_IN_INCHES);
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
