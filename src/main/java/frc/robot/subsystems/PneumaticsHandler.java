package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class PneumaticsHandler {

    private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
    private final Solenoid climberBlockPh = new Solenoid(PneumaticsModuleType.REVPH, 15);

    public PneumaticsHandler() {
        compressor.enableAnalog(70, 120);
    }

    public void lockClimber() {
        climberBlockPh.set(true);
    }

    public void releaseClimber() {
        climberBlockPh.set(false);
    }
    
}
