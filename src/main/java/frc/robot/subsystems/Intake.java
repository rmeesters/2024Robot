package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;

public class Intake extends SubsystemBase {

    private static final String profileDefault = "Default";
    private static final String highSpeed = "High Speed";
    private static final String highAccuracy = "High Accuracy";
    private static final String longRange = "Long Range";
    private final SendableChooser<String> profileChooser = new SendableChooser<>();
    
    private TalonFX fxIntakeMotor;
    private TalonFXConfiguration fxConfig;

    private Rev2mDistanceSensor distanceSensor;

    /**
     * Intake consists of 1 falcon500 motor to move the conveyer belt and the intake wheels.
     */
    public Intake() {
        fxIntakeMotor = new TalonFX(Constants.Intake.Motor.driveMotorID);
        fxConfig = new TalonFXConfiguration();
        fxIntakeMotor.getConfigurator().apply(fxConfig);

        setUpProfileChooser();
        setUpDistanceSensor();
    }

    private void setUpProfileChooser() {
        profileChooser.setDefaultOption("Default", profileDefault);
        profileChooser.addOption("High Speed", highSpeed);
        profileChooser.addOption("High Accuracy", highAccuracy);
        profileChooser.addOption("Long Range", longRange);
        SmartDashboard.putData("Profile", profileChooser);
    }

    private void setUpDistanceSensor() {
        distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);

        distanceSensor.setAutomaticMode(true);
        distanceSensor.setEnabled(true);
        
        switch (profileChooser.getSelected()) {
            case highSpeed -> distanceSensor.setRangeProfile(RangeProfile.kHighSpeed);
            case highAccuracy -> distanceSensor.setRangeProfile(RangeProfile.kHighAccuracy);
            case longRange -> distanceSensor.setRangeProfile(RangeProfile.kLongRange);
            default -> distanceSensor.setRangeProfile(RangeProfile.kDefault);
        }
    }

    public void setSpeed(double speedPercentage) {
        fxIntakeMotor.set(-speedPercentage);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Range", distanceSensor.getRange());
        SmartDashboard.putBoolean("Valid Range", distanceSensor.isRangeValid());
    }

    public boolean inRange(double distance) {
        return distanceSensor.isRangeValid() && distanceSensor.getRange() < distance;
    }
    
}
