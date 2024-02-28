package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class PneumaticsHandler {

    private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
    private final Solenoid s_climberBlock = new Solenoid(PneumaticsModuleType.REVPH, 15);
    private final Solenoid s_shooterBlock = new Solenoid(PneumaticsModuleType.REVPH, 12);
    //private final Solenoid s_pusher = new Solenoid(PneumaticsModuleType.REVPH, 13);

    public PneumaticsHandler() {
        compressor.enableAnalog(70, 120);
    }

    /** Activate/Deactivate climber blocker
     * @param value true: blocked, false: released
     */
    public void setClimber(boolean value) {
        s_climberBlock.set(value);
    }

    public void setShooter(boolean value) {
        s_shooterBlock.set(value);
    }

    // public void setPusher(boolean value) {
    //     s_pusher.set(value);
    // }
    
}
