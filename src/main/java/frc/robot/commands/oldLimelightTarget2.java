// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.LimelightHelpers.*;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.*;
import frc.robot.LimelightHelpers;

public class oldLimelightTarget2 extends Command {
    /** Creates a new LimelightTarget. */
    private Swerve s_Swerve;
    double distancexAway;
    double distanceyAway;
    double limelightMountAngleDegrees = 2;

    double limelightLensHeightCm = 25; // in cm
    double goalheightCm = 25.5;

    double cameraOffset = 5;
    private double px;
    private double py;
    private double KP = 0.025; // was 0.0125
    final private double TOLERANCE_VALUE = 4.0;

    public oldLimelightTarget2() {
        s_Swerve = RobotContainer.s_Swerve;
        addRequirements(s_Swerve);

    }

    public void drivetoTarget() {

        // LimelightHelpers.LimelightResults llresults =
        // LimelightHelpers.getLatestResults("limelight-gnomes");
        // var rrResults = llresults.targetingResults.targets_Retro[0];

        // LimelightTarget_Retro target = new LimelightTarget_Retro();
        // Pose2d targetpos = rrResults.getCameraPose_TargetSpace2D();
        // LimelightResults r = new LimelightResults();

        try {

            distancexAway = LimelightHelpers.getTX("limelight-gnomes"); // target.tx;
            distanceyAway = LimelightHelpers.getTY("limelight-gnomes");
        } catch (Exception e) {
            distancexAway = 0;
            distancexAway = 0;
        }

        new SequentialCommandGroup(
                new InstantCommand(() -> this.DriveXtoTarget()),
                new InstantCommand(() -> this.DriveYtoTarget()));

        // distanceyAway = targetpos.getY();

        // SmartDashboard.putNumber("distance x away from target",
        // r.targetingResults.getBotPose2d().getX());
        // SmartDashboard.putNumber("TX Angle:", distancexAway);
        // SmartDashboard.putNumber("TY Angle:", distanceyAway);
        // SmartDashboard.putNumber("distance y away from target", distanceyAway);

        // can change later.

        // s_Swerve.drive(
        // new Translation2d(0,
        // distancexAway-cameraOffset).times(Constants.Swerve.maxSpeed), //0.125
        // 0 * Constants.Swerve.maxAngularVelocity,
        // false,
        // true
        // //old +0.125
        // );

        // CALC DIST

        // double angletoGoalrad = (limelightMountAngleDegrees + distanceyAway)*(Math.PI
        // /180.0);

        // double distancefromTar= (goalheightCm-
        // limelightLensHeightCm)/Math.tan(angletoGoalrad);

        // SmartDashboard.putNumber("target dist",distancefromTar/10);

    }

    public void DriveYtoTarget() {
        py = Math.abs(distanceyAway * KP);
        if (distanceyAway > 1.0) {
            SmartDashboard.putBoolean("On target", false);
            s_Swerve.drive(
                    new Translation2d(-py, 0).times(Constants.Swerve.maxSpeed), // 0.125
                    0 * Constants.Swerve.maxAngularVelocity,
                    false,
                    true
            // old +0.125
            );
        }

        else if (distancexAway < -1.0) {
            SmartDashboard.putBoolean("On target", false);
            s_Swerve.drive(
                    new Translation2d(py, 0).times(Constants.Swerve.maxSpeed), // 0.125
                    0 * Constants.Swerve.maxAngularVelocity,
                    false,
                    true
            // old +0.125
            );

        }

        else {
            SmartDashboard.putBoolean("On target", true);
            s_Swerve.drive(
                    new Translation2d(0, 0).times(Constants.Swerve.maxSpeed),
                    0 * Constants.Swerve.maxAngularVelocity,
                    false,
                    true);
        }

    }

    public void DriveXtoTarget() {

        px = Math.abs(distancexAway * KP);
        if (distancexAway > 1.0) {
            SmartDashboard.putBoolean("On target", false);
            s_Swerve.drive(
                    new Translation2d(0, -px).times(Constants.Swerve.maxSpeed), // 0.125
                    0 * Constants.Swerve.maxAngularVelocity,
                    false,
                    true
            // old +0.125
            );
        }

        else if (distancexAway < -1.0) {
            SmartDashboard.putBoolean("On target", false);
            s_Swerve.drive(
                    new Translation2d(0, px).times(Constants.Swerve.maxSpeed), // 0.125
                    0 * Constants.Swerve.maxAngularVelocity,
                    false,
                    true
            // old +0.125
            );

        } else {
            SmartDashboard.putBoolean("On target", true);
            s_Swerve.drive(
                    new Translation2d(0, 0).times(Constants.Swerve.maxSpeed),
                    0 * Constants.Swerve.maxAngularVelocity,
                    false,
                    true);
        }

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drivetoTarget();
        SmartDashboard.putBoolean("Limelight Command Execute", true);
        LimelightHelpers.setPipelineIndex("limelight-gnomes", 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Limelight Command Execute", false);
        LimelightHelpers.setPipelineIndex("limelight-gnomes", 1);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
