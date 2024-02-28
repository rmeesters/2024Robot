package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;

public class PneumaticsHandler {

    private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
    private final Solenoid s_climberBlock = new Solenoid(PneumaticsModuleType.REVPH, Constants.Pneumatics.CLIMBER_ID); // 15
    private final Solenoid s_shooterBlock = new Solenoid(PneumaticsModuleType.REVPH, Constants.Pneumatics.SHOOTER_ID); // 12
    //private final Solenoid s_pusher = new Solenoid(PneumaticsModuleType.REVPH, Constants.Pneumatics.PUSHER_ID); // 13

    public PneumaticsHandler() {
        compressor.enableAnalog(Constants.Pneumatics.MIN_PRESSURE, Constants.Pneumatics.MAX_PRESSURE);
    }

    /** Activate/Deactivate climber blocker
     * @param value true: blocked, false: released
     */
    public void setClimber(boolean value) {
        s_climberBlock.set(value);
    }

    /** Activate/Deactivate shooter blocker
     * @param value true: released, false: blocked
     */
    public void setShooter(boolean value) {
        s_shooterBlock.set(value);
    }

    // public void setPusher(boolean value) {
    //     s_pusher.set(value);
    // }
    
}
