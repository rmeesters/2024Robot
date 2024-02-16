package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveWithLimelight;
import frc.robot.commands.IntakeNote;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class AutoLoad extends SequentialCommandGroup {

    private final Intake s_Intake = RobotContainer.s_Intake;

    /**
     * Auto for finding and loading a note into the robot.
     * <hr>
     * 
     * <h3>Instructions:</h3>
     * <ol>
     * <li>Locate note
     * <li>Go to note
     * <li>Rotate front to note
     * <li>Start intake
     * <li>Sense for note
     * <li>Stop after detected
     */
    public AutoLoad() {
        // Reset odometry and follow trajectory
        addCommands(
                new DriveWithLimelight(false),
                new IntakeNote());

    }
    
}
