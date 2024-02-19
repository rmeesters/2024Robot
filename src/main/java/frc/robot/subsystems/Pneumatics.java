package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;

public class Pneumatics {

    private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
    private final Solenoid climberBlockPh = new Solenoid(PneumaticsModuleType.REVPH, 15);

    public Pneumatics() {
        compressor.disable();
        compressor.enableAnalog(70, 120);
        //compressor.enableDigital();
        
        System.out.println(compressor.isEnabled());

        // Get compressor current draw.
        System.out.println(compressor.getCurrent());
        // Get whether the compressor is active.
        System.out.println(compressor.isEnabled());
        // Get the digital pressure switch connected to the PCM/PH.
        // The switch is open when the pressure is over ~120 PSI.
        System.out.println(compressor.getPressureSwitchValue());
    }

    public void lockClimber() {
        climberBlockPh.set(true);
    }

    public void releaseClimber() {
        climberBlockPh.set(false);
    }
    
}
