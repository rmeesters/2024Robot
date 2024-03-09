package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;

public class PneumaticsHandler {

    private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
    private final Solenoid s_climberBlock = new Solenoid(PneumaticsModuleType.REVPH, 15);
    private final Solenoid s_shooterBlock = new Solenoid(PneumaticsModuleType.REVPH, 12);
    private final Solenoid ampSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 14);

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

    public void setShooterSolenoid(boolean on) {
        s_shooterBlock.set(on);
    }
    public void setAmpSolenoid(boolean on) {
        ampSolenoid.set(on);
    }

}
