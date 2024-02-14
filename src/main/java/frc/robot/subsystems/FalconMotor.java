package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Robot;

public class FalconMotor {

    private final int mMotorId;
    private final int mCanCoderId;
    private final Rotation2d mAngleOffset;

    private TalonFX mMotor;
    private CANcoder mCanCoder;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    public FalconMotor(int motor_id, int cancoder_id, Rotation2d angle_offset) {
        mMotorId = motor_id;
        mCanCoderId = cancoder_id;
        mAngleOffset = angle_offset;

        /* Angle Encoder Config */
        mCanCoder = new CANcoder(mCanCoderId);
        mCanCoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        /* Angle Motor Config */
        mMotor = new TalonFX(mMotorId);
        mMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute();
    }

    public FalconMotor(int motor_id) {
        this(motor_id, -1, new Rotation2d(0));
    }

    public void setSpeed(double speed, boolean isOpenLoop) {
        if(isOpenLoop){
            driveDutyCycle.Output = speed / Constants.Swerve.maxSpeed;
            mMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(speed, Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(speed);
            mMotor.setControl(driveVelocity);
        }
    }

    public void resetToAbsolute() {
        double absolutePosition = getCANcoder().getRotations() - mAngleOffset.getRotations();
        mMotor.setPosition(absolutePosition);
    }

    public Rotation2d getCANcoder() {
        return Rotation2d.fromRotations(mCanCoder.getAbsolutePosition().getValue());
    }

}
