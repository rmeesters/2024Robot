package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AngleShooterCommand;
import frc.robot.commands.ArmShooterCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveClimberCommand;
import frc.robot.commands.PrepareAmpCommand;
import frc.robot.commands.ReverseEverythingCommand;
import frc.robot.commands.ReverseIntakeCommand;
import frc.robot.commands.ScoreAmpCommand;
import frc.robot.commands.AutoPrepareAmpCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PneumaticsHandler;
import frc.robot.subsystems.Roller;
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

    public static boolean noteLoaded = false;

    public static final Timer shooterTimer = new Timer();

    /* Controllers */
    // Main Driver Controller
    public static final Joystick driver = new Joystick(0);
    private final GenericHID driver_hid = new GenericHID(0);

    // Side Driver Controller
    private final Joystick coDriver = new Joystick(1);
    private final GenericHID coDriver_hid = new GenericHID(1);

    /* Drive Controls */
    public static final int translationAxis = PS4Controller.Axis.kLeftY.value;
    public static final int strafeAxis = PS4Controller.Axis.kLeftX.value;
    private final int rotationAxis = PS4Controller.Axis.kRightX.value;
    private final int rotationTargetAxis = 3; // RightY: 3;


    /* Driver Buttons */

    // Enable/Disable field relative movement
    private final JoystickButton b_robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    // Set the direction that the robot is facing to be the new forward
    private final JoystickButton b_zeroGyro = new JoystickButton(driver, PS4Controller.Button.kOptions.value);

    // Reset forward back to initial forward direction
    private final JoystickButton b_resetGyro = new JoystickButton(driver, PS4Controller.Button.kPS.value);

    // Set the current position of the shooter to be the new zero
    private final JoystickButton b_zeroShooter = new JoystickButton(driver, PS4Controller.Button.kShare.value);


    /* Intake Buttons */

    // Spin the intake wheels and conveyer belt (1 motor)
    private final JoystickButton b_spinIntake = new JoystickButton(driver, PS4Controller.Button.kL2.value);

    // Spin the intake wheels and conveyer belt backward
    private final JoystickButton b_reverseIntake = new JoystickButton(driver, PS4Controller.Button.kL1.value);

    // Spin the intake wheels and conveyer belt backward
    private final JoystickButton b_reverseEverything_coDriver = new JoystickButton(coDriver, 3); // Square: 3

    /* Shooter Buttons */

    /**
     * Spin the two shooter wheels to full speed (2 motors)
     * Release the shooter pin (1 pneumatic) and start intake (1 motor) after the shooters obtain near full speed or a delay has passed
     * @see Constants.Shooter Shooter Constants
     */
    private final JoystickButton b_spinShooter = new JoystickButton(driver, PS4Controller.Button.kR2.value);
    private final Trigger b_spinShooter_coDriver = new Trigger(coDriver.axisGreaterThan(3, 0.5, CommandScheduler.getInstance().getActiveButtonLoop()));

    // Reverse intake (1 motor) and shooter (2 motors) and lower shooter pin (1 pneumatic)
    private final JoystickButton b_armShooter = new JoystickButton(driver, PS4Controller.Button.kR1.value);
    private final JoystickButton b_armShooter_coDriver = new JoystickButton(coDriver, PS4Controller.Button.kR1.value);

    // Angle shooter down by rotating worm drive (1 motor) until a min/max canCoder is reached (slow down when close)
    private final JoystickButton b_angleDown = new JoystickButton(driver, PS4Controller.Button.kSquare.value);

    // Angle shooter up by rotating worm drive (1 motor) until a min/max canCoder is reached (slow down when close)
    private final JoystickButton b_angleUp = new JoystickButton(driver, PS4Controller.Button.kTriangle.value);

    // Angle shooter to zero by rotating worm drive (1 motor)
    private final JoystickButton b_angleZero = new JoystickButton(driver, PS4Controller.Button.kCross.value);

    private final JoystickButton b_autoPrepareAmp = new JoystickButton(coDriver, 2); // Circle: 2
    private final JoystickButton b_prepareAmp = new JoystickButton(coDriver, PS4Controller.Button.kL1.value);
    private final Trigger b_scoreAmp = new Trigger(coDriver.axisGreaterThan(2, 0.5, CommandScheduler.getInstance().getActiveButtonLoop()));

    /* Climber Buttons */

    // Raise robot by pulling down claws
    private final POVButton b_climbUp = new POVButton(driver_hid, 0); // 0: up

    // Lower robot down by letting up claws
    private final POVButton b_climbDown = new POVButton(driver_hid, 180); // 180: down

    // Raise and Lower trap pneumatic
    private final POVButton b_raiseTrap = new POVButton(coDriver_hid, 0); // 0: up
    private final POVButton b_lowerTrap = new POVButton(coDriver_hid, 180); // 180: down


    /* Handlers */
    public static final PneumaticsHandler h_pneumatics = new PneumaticsHandler();

    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public static final Shooter s_Shooter = new Shooter();
    public static final Intake s_Intake = new Intake();
    public static final Climber s_Climber = new Climber();
    public static final Roller s_Roller = new Roller();

    /* Sendable Chooser and Autonomus Commands */
    private static SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configurePathPlannerCommands();
        s_Swerve.configureAutoBuilder();

        /* Setup */
        setUpSwerveController();
        configureLimelight(Constants.Limelight.Front.NAME);
        configureLimelight(Constants.Limelight.Back.NAME);
        configureButtonBindings();
        configureAutoChooser();
        stopMotors();
        preparePneumatics();
    }

    private void setUpSwerveController() {
        s_Swerve.setDefaultCommand(new TeleopSwerve(
                s_Swerve,
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                () -> -driver.getRawAxis(rotationAxis),
                () -> -driver.getRawAxis(rotationTargetAxis),
                () -> b_robotCentric.getAsBoolean()));
    }

    private void configureLimelight(String limelightName) {
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
        b_resetGyro.onTrue(new InstantCommand(() -> s_Swerve.directGyroForward()));
        b_zeroShooter.onTrue(new InstantCommand(() -> s_Shooter.setZero()));

        /* Intake Buttons */

        b_spinIntake.whileTrue(new IntakeCommand());
        b_reverseIntake.whileTrue(new ReverseIntakeCommand());
        b_reverseEverything_coDriver.whileTrue(new ReverseEverythingCommand());

        /* Shooter Buttons */

        b_spinShooter.whileTrue(new ShootCommand());
        b_spinShooter_coDriver.whileTrue(new ShootCommand());

        /* Angle Buttons */
        
        b_armShooter.whileTrue(new ArmShooterCommand());
        b_armShooter_coDriver.whileTrue(new ArmShooterCommand());

        b_angleUp.whileTrue(new AngleShooterCommand(-1));
        b_angleDown.whileTrue(new AngleShooterCommand(1));
        
        b_climbUp.whileTrue(new MoveClimberCommand(1));
        b_climbDown.whileTrue(new MoveClimberCommand(-1));

        b_angleZero.onTrue(new InstantCommand(() -> s_Shooter.setShaftRotation(0)));

        /* Amp */

        b_autoPrepareAmp.toggleOnTrue(new AutoPrepareAmpCommand());
        
        b_prepareAmp.whileTrue(new PrepareAmpCommand());
        b_scoreAmp.whileTrue(new ScoreAmpCommand());

        b_raiseTrap.onTrue(new InstantCommand(() -> h_pneumatics.setTrapSolenoid(true)));
        b_lowerTrap.onTrue(new InstantCommand(() -> h_pneumatics.setTrapSolenoid(false)));
    }

    private void configurePathPlannerCommands() {
        NamedCommands.registerCommand("Start Shooter", new InstantCommand(() -> s_Shooter.setSpeed(1)));
        NamedCommands.registerCommand("Reverse Shooter", new InstantCommand(() -> s_Shooter.setSpeed(-1)));
        NamedCommands.registerCommand("Stop Shooter", new InstantCommand(() -> s_Shooter.setSpeed(0)));
        NamedCommands.registerCommand("Start Intake", new InstantCommand(() -> s_Intake.setSpeed(1)));
        NamedCommands.registerCommand("Reverse Intake", new InstantCommand(() -> s_Intake.setSpeed(-1)));
        NamedCommands.registerCommand("Stop Intake", new InstantCommand(() -> s_Intake.setSpeed(0)));
        NamedCommands.registerCommand("Lower Shooter Pin", new InstantCommand(() -> h_pneumatics.setShooterSolenoid(true)));
        NamedCommands.registerCommand("Raise Shooter Pin", new InstantCommand(() -> h_pneumatics.setShooterSolenoid(false)));
        NamedCommands.registerCommand("Angle to 0", new InstantCommand(() -> s_Shooter.setShaftRotation(0)));
        NamedCommands.registerCommand("Angle to 1.5", new InstantCommand(() -> s_Shooter.setShaftRotation(1.5)));
        NamedCommands.registerCommand("Angle to 10.7", new InstantCommand(() -> s_Shooter.setShaftRotation(10.7)));
        NamedCommands.registerCommand("Angle to 11", new InstantCommand(() -> s_Shooter.setShaftRotation(11)));
        NamedCommands.registerCommand("Tilt Up", new InstantCommand(() -> h_pneumatics.setTiltSolenoid(false)));
        NamedCommands.registerCommand("Tilt Down", new InstantCommand(() -> h_pneumatics.setTiltSolenoid(true)));
    }

    private void configureAutoChooser() {
        // Autonomous Sendable Chooser
        autoChooser = AutoBuilder.buildAutoChooser("Middle Side - 3 Note");

        SmartDashboard.putData("Selected Auto", autoChooser);
    }

    private void stopMotors() {
        s_Shooter.setSpeed(0);
        s_Intake.setSpeed(0);
        s_Roller.setSpeed(0);
        s_Climber.setSpeed(0);
    }

    private void preparePneumatics() {
        h_pneumatics.setShooterSolenoid(false);
        h_pneumatics.setClimberSolenoid(true);
        h_pneumatics.setTrapSolenoid(false);
        h_pneumatics.setTiltSolenoid(false);
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
