package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveToTag;
import frc.robot.commands.ShootNote;
import frc.robot.subsystems.Intake;

public class AutoShoot extends SequentialCommandGroup {
    
    /**
     * Auto for finding and loading a note into the robot.
     * <hr>
     * 
     * <h3>Instructions:</h3>
     * <ol>
     * <li>TODO
     */
    public AutoShoot() {
        // Reset odometry and follow trajectory
        //TODO automatically decide and give priority to what pipeline to use
        addCommands(
                //new DriveToTag(Constants.Limelight.Pipelines.SPEAKER),
                new ShootNote());

    }

}
