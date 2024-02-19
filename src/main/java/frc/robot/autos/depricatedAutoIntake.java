package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.depricatedDriveToNote;
import frc.robot.commands.IntakeNote;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class depricatedAutoIntake extends ParallelCommandGroup {

    private final Intake s_Intake = RobotContainer.s_Intake;

    /**
     * Auto for finding and loading a note into the robot.
     * <hr>
     * 
     * <h3>Instructions:</h3>
     * <ol>
     * <li>Locate note
     * <li>Go to note
     * <li>Pick up note
     */
    public depricatedAutoIntake() {
        // Reset odometry and follow trajectory
        // TODO make IntakeNote start 1 second before DriveWithLimelight ends
        addCommands(
                //new AutoDrive(),
                new IntakeNote());

    }
    
}
