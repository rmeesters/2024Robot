package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class SwerveCommand extends Command {
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private DoubleSupplier rotationTargetSup;
    private BooleanSupplier robotCentricSup;

    public SwerveCommand(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier rotationSup, DoubleSupplier rotationTargetSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.rotationTargetSup = rotationTargetSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEAD_BAND);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEAD_BAND);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.STICK_DEAD_BAND);
        double rotationTargetVal = MathUtil.applyDeadband(rotationTargetSup.getAsDouble(), 0.9);

        if (rotationTargetVal != 0) {
            double yaw = s_Swerve.gyro.getYaw();
            double targetRotation = rotationTargetVal > 0 ? (yaw > 180 ? 360 : 0) : 180;
            double distance = yaw - targetRotation;
            SmartDashboard.putNumber("rotationDistance", distance);

            rotationVal = Math.min(0.5, Math.max(-0.5, distance / 6));
        }

        /* Drive */
        s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.MAX_SPEED),
                rotationVal * Constants.Swerve.MAX_ANGULAR_VELOCITY,
                !robotCentricSup.getAsBoolean(),
                true);
    }
}