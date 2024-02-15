package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

//import edu.wpi.first.wpilibj.I2C.Port; //TODO This Crashes the Robot
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//import com.revrobotics.*;

public class Intake extends SubsystemBase {
    
    private TalonFX fxIntakeMotor;
    private TalonFXConfiguration fxConfig;

    //private ColorSensorV3 distanceSensor;

    /**
     * Intake consists of 1 falcon500 motor to move the conveyer belt and the intake wheels.
     */
    public Intake() {
        fxIntakeMotor = new TalonFX(Constants.Intake.Motor.driveMotorID);
        fxConfig = new TalonFXConfiguration();
        fxIntakeMotor.getConfigurator().apply(fxConfig);

        //distanceSensor = new ColorSensorV3(Port.kOnboard);
    }

    public void setSpeed(double speed) {
        fxIntakeMotor.set(-speed / Constants.Intake.Motor.maxSpeed);
    }

    @Override
    public void periodic(){
        //SmartDashboard.putNumber("REV Distance Sensor", distanceSensor.getProximity());
    }
    
}
