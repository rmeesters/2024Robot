package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared.
 * 
 * Since Command-based is a "declarative" paradigm,
 * very little robot logic should actually be handled in
 * the {@link Robot} periodic methods (other than the scheduler calls).
 * 
 * Instead, the structure of the robot (subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = PS4Controller.Axis.kLeftY.value;
    private final int strafeAxis = PS4Controller.Axis.kLeftX.value;
    private final int rotationAxis = PS4Controller.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, PS4Controller.Button.kTriangle.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    private final JoystickButton testMovement = new JoystickButton(driver, PS4Controller.Button.kCross.value);

    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();

    /* Sendable Chooser and Autonomus Commands */
    private static SendableChooser<Command> autoChooser;

    /**
     * Test Auto
     * 1. move forward 1 meter
     * 2. wait 1 second
     * 3. move back 1 meter
     */
    private final Command test1_auto = new SequentialCommandGroup(
            new AutoDrive(List.of(
                    new Pose2d(0, 0, new Rotation2d(0)),
                    new Pose2d(1, 0, new Rotation2d(0))), false),
            new WaitCommand(1),
            new AutoDrive(List.of(
                    new Pose2d(1, 0, new Rotation2d(0)),
                    new Pose2d(0, 0, new Rotation2d(0))), false));

    /**
     * Second Test Auto
     * 1. move forward and right 1 meter
     * 2. move forward 1 meter
     * 3. move back to start
     */
    private final Command test2_auto = new SequentialCommandGroup(
            new AutoDrive(List.of(
                    new Pose2d(0, 0, new Rotation2d(0)),
                    new Pose2d(1, 1, new Rotation2d(0))), false),
            new AutoDrive(List.of(
                    new Pose2d(1, 1, new Rotation2d(0)),
                    new Pose2d(2, 1, new Rotation2d(0))), false),
            new AutoDrive(List.of(
                    new Pose2d(2, 1, new Rotation2d(0)),
                    new Pose2d(0, 0, new Rotation2d(0))), false));

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> robotCentric.getAsBoolean()));

        // Configure the button bindings
        configureButtonBindings();

        configureAutoChooser();
    }

    /**
     * Use this method to define your button mappings.
     * 
     * Buttons can be created by instantiating a {@link GenericHID}
     * or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        testMovement.onTrue(new AutoDrive(List.of(
                new Pose2d(0, 0, new Rotation2d(0)),
                new Pose2d(1, 0, new Rotation2d(0)),
                new Pose2d(1, 2, new Rotation2d(0))), false));
    }

    private void configureAutoChooser() {
        // Autonomous Sendable Chooser
        autoChooser = new SendableChooser<Command>();
        autoChooser.addOption("X Auto", test1_auto);
        autoChooser.addOption("XY Auto", test2_auto);

        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
