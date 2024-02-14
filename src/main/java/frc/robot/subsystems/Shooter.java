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

    public void setSpeed(double speed, boolean isOpenLoop) {
        mLeftShooterMotor.setSpeed(speed, isOpenLoop);
        mRightShooterMotor.setSpeed(-speed, isOpenLoop);
    }

    public void setAngle(double degrees) {
        double changeInDistance = getBoltAdjustment(degrees);
        double rotations = Constants.Shooter.boltGrovesPerInch * changeInDistance;
        // double speed = Constants.Shooter.shaftCircumference * Constants.Shooter.boltGrovesPerInch;
        // mAngleMotor.setSpeed(speed, false);
    }

    /**
     * Get the distance the base of the arm must travel
     * @param degrees angle starting from the horizontal facing the back
     * @return The adjustment in inches
     */
    private double getBoltAdjustment(double degrees) {
        final double A = -0.0156853;
        final double B = 0.166208;
        final double C = -6.04656;
        final double D = 97.6943;

        double position = A*Math.pow(degrees, 3) + B*Math.pow(degrees, 2) + C*degrees + D;

        return position - boltPosition;
    }

    //TEST METHOD
    public void test() {
        setSpeed(1.0 * Constants.Swerve.wheelCircumference / 60, false);
    }

}
