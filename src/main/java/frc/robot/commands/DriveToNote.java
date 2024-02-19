package frc.robot.commands;

import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.autos.AutoDrive;
import frc.robot.subsystems.Swerve;

public class DriveToNote extends Command {

    private final Swerve s_Swerve = RobotContainer.s_Swerve;

    private boolean NOTE_IS_VISIBLE;

    private AutoDrive driveAuto;
    // private IntakeNote intakeCommand;

    // private boolean startedIntake = false;

    private Timer timer = new Timer();

    /**
     * Drive to note using front limelight
     */
    public DriveToNote() {

    }

    /**
     * The initial subroutine of a command. Called once when the command is
     * initially scheduled.
     */
    @Override
    public void initialize() {
        // Stop if target is not visible
        NOTE_IS_VISIBLE = LimelightHelpers.getTV(Constants.Limelight.Front.NAME);
        if (!NOTE_IS_VISIBLE) {
            System.out.println("No target visible");
            cancel();
            return;
        }

        List<Pose2d> points = calculatePoints();
        driveAuto = new AutoDrive(points, false);
        //intakeCommand = new IntakeNote();
        
        timer.restart();
        driveAuto.initialize();
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        driveAuto.execute();

        // if (timer.hasElapsed(driveAuto.timeToFinish() - 1)) {
        //     if (startedIntake) {
        //         intakeCommand.execute();
        //         return;
        //     }
        //     intakeCommand.initialize();
        //     startedIntake = true;
        // }
    }

    /**
     * The action to take when the command ends.
     * Called either when the command finishes normally or interrupted/canceled.
     *
     * Do not schedule commands here that share requirements with this command.
     * Use {@link #andThen(Command...)} instead.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        if (!NOTE_IS_VISIBLE) {
            return;
        }

        driveAuto.end(interrupted);
        //intakeCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return !NOTE_IS_VISIBLE || driveAuto.isFinished();
    }

    private List<Pose2d> calculatePoints() {
        double TX = Math.toRadians(s_Swerve.getGyroYaw().getDegrees() + LimelightHelpers.getTX(Constants.Limelight.Back.NAME));
        double TY = Math.toRadians(Constants.Limelight.Back.CAMERA_ANGLE + LimelightHelpers.getTY(Constants.Limelight.Back.NAME));

        double distance = Constants.Limelight.Back.CAMERA_HEIGHT / Math.tan(TY);

        double dx = distance * Math.cos(TX);
        double dy = distance * Math.sin(TX);

        return List.of(
                new Pose2d(0, 0, s_Swerve.getGyroYaw()),
                new Pose2d(dx, dy, new Rotation2d(0)));
    }
}
