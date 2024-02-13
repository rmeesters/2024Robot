package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    
    public FalconMotor mLeftShooterMotor;
    public FalconMotor mRightShooterMotor;
    public FalconMotor mAngleMotor;

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

    }

}
