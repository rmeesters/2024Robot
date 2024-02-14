package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    
    public FalconMotor mLeftShooterMotor;
    public FalconMotor mRightShooterMotor;
    public FalconMotor mAngleMotor;

    public double boltPosition;

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

    // public void spinToRPM(double targetRPM) {
    //     setSpeed(targetRPM * Constants.Swerve.wheelCircumference / 60, false);
    // }

    public void setShooterSpeed(double speed, boolean isOpenLoop) {
        mLeftShooterMotor.setSpeed(speed, isOpenLoop);
        mRightShooterMotor.setSpeed(-speed, isOpenLoop);
    }

    public void stopShooter() {
        mLeftShooterMotor.setSpeed(0, true);
        mRightShooterMotor.setSpeed(0, true);
    }

    // public void setAngle(double degrees) {
    //     double changeInDistance = getBoltAdjustment(degrees);
    //     double rotations = Constants.Shooter.boltGrovesPerInch * changeInDistance;
    //     // double speed = Constants.Shooter.shaftCircumference * Constants.Shooter.boltGrovesPerInch;
    //     // mAngleMotor.setSpeed(speed, false);
    // }

    public void setAngleSpeed(double speed, boolean isOpenLoop) {
        mAngleMotor.setSpeed(speed, isOpenLoop);
    }

    public void stopAngleAdjustment() {
        mAngleMotor.setSpeed(0, false);
    }

}
