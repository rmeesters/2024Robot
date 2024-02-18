package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.autos.AutoDrive;
import frc.robot.commands.DriveToNote;
import frc.robot.commands.DriveToTag;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.ShootNote;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared.
 * 
 * Since Command-based is a "declarative" paradigm,
 * very little robot logic should actually be handled in
 * the {@link Robot} periodic methods (other than the scheduler calls).
 * 
 * Instead, the structure of the robot (subsystems, commands, and button
 * mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final GenericHID pov = new GenericHID(0);

    /* Drive Controls */
    private final int translationAxis = PS4Controller.Axis.kLeftY.value;
    private final int strafeAxis = PS4Controller.Axis.kLeftX.value;
    private final int rotationAxis = PS4Controller.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, PS4Controller.Button.kOptions.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Intake */
    private final JoystickButton spinIntake = new JoystickButton(driver, PS4Controller.Button.kL2.value);

    /* Shooter */
    private final JoystickButton spinShooter = new JoystickButton(driver, PS4Controller.Button.kR2.value);
    private final JoystickButton angleDown = new JoystickButton(driver, PS4Controller.Button.kL1.value);
    private final JoystickButton angleUp = new JoystickButton(driver, PS4Controller.Button.kR1.value);

    private final JoystickButton test_setAngleHigh = new JoystickButton(driver, PS4Controller.Button.kSquare.value);
    private final JoystickButton test_setAngleLow = new JoystickButton(driver, PS4Controller.Button.kTriangle.value);
    private final JoystickButton intakeNote = new JoystickButton(driver, PS4Controller.Button.kCircle.value);
    private final JoystickButton fireNote = new JoystickButton(driver, PS4Controller.Button.kCross.value);

    /* Climber */
    private final POVButton climbUp = new POVButton(pov, 0);
    private final POVButton climbDown = new POVButton(pov, 180);

    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public static final Shooter s_Shooter = new Shooter();
    public static final Intake s_Intake = new Intake();
    public static final Climber s_Climber = new Climber();

    /* Sendable Chooser and Autonomus Commands */
    private static SendableChooser<Command> autoChooser;

    /**
     * Example Auto
     * 1. move forward 1 meter
     * 2. wait 1 second
     * 3. move back 1 meter
     */
    private final Command exampleAuto = new SequentialCommandGroup(
            new AutoDrive(List.of(
                    new Pose2d(0, 0, new Rotation2d(0)),
                    new Pose2d(1, 0, new Rotation2d(0))), false),
            new WaitCommand(3),
            new AutoDrive(List.of(
                    new Pose2d(1, 0, new Rotation2d(0)),
                    new Pose2d(0, 0, new Rotation2d(0))), true));

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureLimelight(Constants.Limelight.Front.NAME);
        configureLimelight(Constants.Limelight.Back.NAME);

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

    private void configureLimelight(String limelightName) {
        // LimelightHelpers.setLEDMode_PipelineControl(limelightName);
        LimelightHelpers.setLEDMode_ForceOn(limelightName);
        LimelightHelpers.setStreamMode_Standard(limelightName);
        LimelightHelpers.setCameraMode_Processor(limelightName);
    }

    /**
     * Use this method to define your button mappings.
     * 
     * Buttons can be created by instantiating a {@link GenericHID}
     * or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
     * {@link XboxController}),
     * and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        angleDown.onTrue(new InstantCommand(() -> s_Shooter.setShaftSpeed(1)));
        angleDown.onFalse(new InstantCommand(() -> s_Shooter.setShaftSpeed(0)));

        angleUp.onTrue(new InstantCommand(() -> s_Shooter.setShaftSpeed(-1)));
        angleUp.onFalse(new InstantCommand(() -> s_Shooter.setShaftSpeed(0)));

        spinShooter.onTrue(new InstantCommand(() -> s_Shooter.setSpeed(1)));
        spinShooter.onFalse(new InstantCommand(() -> s_Shooter.setSpeed(0)));

        spinIntake.onTrue(new InstantCommand(() -> s_Intake.setSpeed(0.3)));
        spinIntake.onFalse(new InstantCommand(() -> s_Intake.setSpeed(0)));

        test_setAngleHigh.onTrue(new InstantCommand(() -> s_Shooter.setShaftRotation(0)));
        test_setAngleLow.onTrue(new InstantCommand(() -> s_Shooter.setShaftRotation(Constants.Shooter.canCoderLimit)));

        intakeNote.onTrue(new IntakeNote());
        fireNote.onTrue(new ShootNote());

        climbUp.onTrue(new InstantCommand(() -> s_Climber.setSpeed(10)));
        climbUp.onFalse(new InstantCommand(() -> s_Climber.setSpeed(0)));

        climbDown.onTrue(new InstantCommand(() -> s_Climber.setSpeed(-10)));
        climbDown.onFalse(new InstantCommand(() -> s_Climber.setSpeed(0)));
    }

    private void configureAutoChooser() {
        // Autonomous Sendable Chooser
        autoChooser = new SendableChooser<Command>();
        autoChooser.setDefaultOption("Move Auto", exampleAuto);
        autoChooser.addOption("Load Note", new IntakeNote());
        autoChooser.addOption("Drive to Note", new DriveToNote());
        autoChooser.addOption("Drive to Tag", new DriveToTag(
                Constants.Limelight.Pipelines.Speaker.Red.CENTER,
                Constants.Limelight.Pipelines.Speaker.APRILTAG_HEIGHT));

        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
