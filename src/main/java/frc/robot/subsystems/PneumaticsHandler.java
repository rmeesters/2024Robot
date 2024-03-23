package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;

public class PneumaticsHandler {

    private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
    private final Solenoid s_climberBlock = new Solenoid(PneumaticsModuleType.REVPH, 15);
    private final Solenoid s_shooterBlock = new Solenoid(PneumaticsModuleType.REVPH, 12);
    private final Solenoid s_tiltSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 13);
    private final Solenoid s_ampSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 14);

    public PneumaticsHandler() {
        compressor.enableAnalog(Constants.Pneumatics.MIN_PRESSURE, Constants.Pneumatics.MAX_PRESSURE);
    }

    /** Activate/Deactivate climber blocker
     * @param value true: blocked, false: released
     */
    public void setClimberSolenoid(boolean value) {
        s_climberBlock.set(value);
    }

    /** Activate/Deactivate shooter blocker
     * @param value true: released, false: blocked
     */
    public void setShooterSolenoid(boolean value) {
        s_shooterBlock.set(value);
    }
    
    public void setAmpSolenoid(boolean value) {
        s_ampSolenoid.set(value);
    }

    public void setTiltSolenoid(boolean value) {
        s_tiltSolenoid.set(value);
    }

}
