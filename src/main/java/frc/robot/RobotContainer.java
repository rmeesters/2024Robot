package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PneumaticsHandler;
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
    private final JoystickButton b_zeroGyro = new JoystickButton(driver, PS4Controller.Button.kOptions.value);
    private final JoystickButton b_robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Intake */
    private final JoystickButton b_spinIntake = new JoystickButton(driver, PS4Controller.Button.kL2.value);

    /* Shooter */
    private final JoystickButton b_spinShooter = new JoystickButton(driver, PS4Controller.Button.kR2.value);
    private final JoystickButton b_backupIntake = new JoystickButton(driver, PS4Controller.Button.kL1.value);
    private final JoystickButton b_armShooter = new JoystickButton(driver, PS4Controller.Button.kR1.value);

    private final JoystickButton b_angleDown = new JoystickButton(driver, PS4Controller.Button.kSquare.value);
    private final JoystickButton b_angleUp = new JoystickButton(driver, PS4Controller.Button.kTriangle.value);
    private final JoystickButton b_setShootPosition = new JoystickButton(driver, PS4Controller.Button.kCircle.value);
    private final JoystickButton b_setPickupPosition = new JoystickButton(driver, PS4Controller.Button.kCross.value);

    /* Climber */
    private final POVButton b_climbUp = new POVButton(pov, 0);
    private final POVButton b_climbDown = new POVButton(pov, 180);

    private final POVButton b_lockClimber = new POVButton(pov, 90);
    private final POVButton b_releaseClimber = new POVButton(pov, 270);

    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public static final Shooter s_Shooter = new Shooter();
    public static final Intake s_Intake = new Intake();
    public static final Climber s_Climber = new Climber();

    public static final PneumaticsHandler h_pneumatics = new PneumaticsHandler();

    /* Sendable Chooser and Autonomus Commands */
    private static SendableChooser<Command> autoChooser;

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
                        () -> b_robotCentric.getAsBoolean()));

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
        b_zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        b_spinShooter.onTrue(new InstantCommand(() -> s_Shooter.setSpeed(1)));
        b_spinShooter.onFalse(new InstantCommand(() -> s_Shooter.setSpeed(0)));

        /* Intake */
        b_spinIntake.onTrue(new InstantCommand(() -> s_Intake.setSpeed(0.6)));
        b_spinIntake.onFalse(new InstantCommand(() -> s_Intake.setSpeed(0)));

        b_backupIntake.onTrue(new InstantCommand(() -> s_Intake.setSpeed(-0.6)));
        b_backupIntake.onFalse(new InstantCommand(() -> s_Intake.setSpeed(0)));

        /* Shooter Angle */
        b_angleUp.onTrue(new InstantCommand(() -> s_Shooter.setShaftSpeed(-1)));
        b_angleUp.onFalse(new InstantCommand(() -> s_Shooter.setShaftSpeed(0)));

        b_angleDown.onTrue(new InstantCommand(() -> s_Shooter.setShaftSpeed(1)));
        b_angleDown.onFalse(new InstantCommand(() -> s_Shooter.setShaftSpeed(0)));

        //b_armShooter.onTrue(new InstantCommand(() -> s_Shooter.focus()));

        b_setShootPosition.onTrue(new InstantCommand(() -> s_Shooter.setShaftPosition(4)));
        b_setPickupPosition.onTrue(new InstantCommand(() -> s_Shooter.setShaftPosition(6)));

        /* Climber */
        b_climbUp.onTrue(new InstantCommand(() -> s_Climber.setSpeed(10)));
        b_climbUp.onFalse(new InstantCommand(() -> s_Climber.setSpeed(0)));

        b_climbDown.onTrue(new InstantCommand(() -> s_Climber.setSpeed(-10)));
        b_climbDown.onFalse(new InstantCommand(() -> s_Climber.setSpeed(0)));

        b_lockClimber.onTrue(new InstantCommand(() -> h_pneumatics.lockClimber()));
        b_releaseClimber.onTrue(new InstantCommand(() -> h_pneumatics.releaseClimber()));
    }

    private void configureAutoChooser() {
        // Autonomous Sendable Chooser
        autoChooser = new SendableChooser<Command>();

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
