package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    
    public FalconMotor mLeftShooterMotor;
    public FalconMotor mRightShooterMotor;
    public FalconMotor mAngleMotor;

    private double boltPosition;

    /**
     * Shooter consists of 3 falcon500 motors, 2 to accellerate the projectile, and 1 to angle the shooter.
     */
    public Shooter() {
        mLeftShooterMotor = new FalconMotor(Constants.Shooter.LeftMotor.driveMotorID);
        mRightShooterMotor = new FalconMotor(Constants.Shooter.RightMotor.driveMotorID);
        mAngleMotor = new FalconMotor(
                Constants.Shooter.AngleMotor.driveMotorID,
                Constants.Shooter.AngleMotor.canCoderID,
                Constants.Shooter.AngleMotor.angleOffset);
        boltPosition = Constants.Shooter.ArmRange;
    }

    public void spinToRPM(double targetRPM) {
        setSpeed(targetRPM * Constants.Swerve.wheelCircumference / 60, false);
    }

    public void setSpeed(double metersPerSecond, boolean isOpenLoop) {
        mLeftShooterMotor.setSpeed(metersPerSecond, isOpenLoop);
        mRightShooterMotor.setSpeed(-metersPerSecond, isOpenLoop);
    }

    public void setAngle(double degrees) {
        double horizontal_distance = getBoltAdjustment(degrees);
        double metersPerSecond = Constants.Swerve.wheelCircumference * Constants.Shooter.boltGrovesPerInch * horizontal_distance;
        mAngleMotor.setSpeed(metersPerSecond, false);
    }

    private double getBoltAdjustment(double degrees) {
        return 0;
    }

    //TEST METHOD
    public void test() {
        setSpeed(1.0 * Constants.Swerve.wheelCircumference / 60, false);
    }

}
