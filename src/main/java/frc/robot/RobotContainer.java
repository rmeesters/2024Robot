package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.DriveClimber;
import frc.robot.autos.DriverAutoMain;
import frc.robot.autos.DriverAutoMidTwoNote;
import frc.robot.autos.DriverAutoMoveBack;
import frc.robot.autos.DriverAutoNoMove;
import frc.robot.autos.DriverAutoWallSide;
import frc.robot.autos.SpitAndMove;
import frc.robot.commands.AngleShooterCommand;
import frc.robot.commands.DriveToNote;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RotateCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TargetSpeakerCommand;
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

    public static Rotation2d gyro_temp = new Rotation2d();
    public static boolean noteLoaded = false;

    /* Controllers */
    public static final Joystick driver = new Joystick(0);
    private final GenericHID pov = new GenericHID(0);

    /* Drive Controls */
    public static final int translationAxis = PS4Controller.Axis.kLeftY.value;
    public static final int strafeAxis = PS4Controller.Axis.kLeftX.value;
    private final int rotationAxis = PS4Controller.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton b_zeroGyro = new JoystickButton(driver, PS4Controller.Button.kOptions.value);
    private final JoystickButton b_robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton b_zeroShooter = new JoystickButton(driver, PS4Controller.Button.kShare.value);
    private final JoystickButton b_forwardGyro = new JoystickButton(driver, PS4Controller.Button.kPS.value);
    private final POVButton b_ampPosition = new POVButton(pov, 90);
    private final POVButton b_ampScore = new POVButton(pov, 270);

    /* Intake */
    private final JoystickButton b_spinIntake = new JoystickButton(driver, PS4Controller.Button.kL2.value);
    private final JoystickButton b_backupIntake = new JoystickButton(driver, PS4Controller.Button.kL1.value);

    /* Shooter */
    private final JoystickButton b_spinShooter = new JoystickButton(driver, PS4Controller.Button.kR2.value);
    // private final JoystickButton b_focusShooter = new JoystickButton(driver, PS4Controller.Button.kR1.value);

    /* Angle */
    private final JoystickButton b_angleDown = new JoystickButton(driver, PS4Controller.Button.kSquare.value);
    private final JoystickButton b_angleUp = new JoystickButton(driver, PS4Controller.Button.kTriangle.value);
    // private final JoystickButton b_setShootPosition = new JoystickButton(driver, PS4Controller.Button.kCircle.value);
    // private final JoystickButton b_setPickupPosition = new JoystickButton(driver, PS4Controller.Button.kCross.value);

    private final JoystickButton b_prepareSpeaker = new JoystickButton(driver, PS4Controller.Button.kCircle.value);

    /* Climber */
    private final POVButton b_climbUp = new POVButton(pov, 0);
    private final POVButton b_climbDown = new POVButton(pov, 180);


    /* Handlers */
    public static final PneumaticsHandler h_pneumatics = new PneumaticsHandler();

    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public static final Shooter s_Shooter = new Shooter();
    public static final Intake s_Intake = new Intake();
    public static final Climber s_Climber = new Climber();

    /* Sendable Chooser and Autonomus Commands */
    private static SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configurePathPlannerCommands();
        s_Swerve.configureAutoBuilder();

        /* Controller Movement */
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> b_robotCentric.getAsBoolean()));

        /* Setup */
        configureLimelight(Constants.Limelight.Front.NAME);
        configureLimelight(Constants.Limelight.Back.NAME);
        configureButtonBindings();
        configureAutoChooser();
        stopMotors();
        preparePneumatics();
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
        b_forwardGyro.onTrue(new InstantCommand(() -> s_Swerve.directGyroForward()));

        /* Intake */
        b_spinIntake.whileTrue(new IntakeCommand());

        /* Shooter */

        b_backupIntake.whileTrue(new InstantCommand(() -> {
            s_Shooter.setSpeed(-1);
            s_Intake.setSpeed(-1);
            h_pneumatics.setShooterSolenoid(true);
        }));
        b_backupIntake.onFalse(new InstantCommand(() -> {
            s_Shooter.setSpeed(0);
            s_Intake.setSpeed(0);
            h_pneumatics.setShooterSolenoid(false);
        }));

        b_spinShooter.whileTrue(new ShootCommand());

        /* Shooter Angle */
        b_zeroShooter.onTrue(new InstantCommand(() -> s_Shooter.setShaftRotation(0)));
        b_ampPosition.onTrue(new InstantCommand(() -> s_Shooter.setShaftRotation(-29.76)));

        b_ampScore.whileTrue(new SequentialCommandGroup(
            new InstantCommand(() -> h_pneumatics.setAmpSolenoid(true)),
            new WaitCommand(0.5),
            new InstantCommand(() -> {
                h_pneumatics.setShooterSolenoid(true);
                s_Intake.setSpeed(0.3);
                s_Shooter.setSpeed(0.2);
        })));
        b_ampScore.onFalse(new InstantCommand(() -> {
            h_pneumatics.setShooterSolenoid(false);
            s_Shooter.setSpeed(0);
            h_pneumatics.setAmpSolenoid(false);
            s_Intake.setSpeed(0);
        }));

        b_angleUp.whileTrue(new AngleShooterCommand(-1));
        b_angleDown.whileTrue(new AngleShooterCommand(1));

        // b_focusShooter.whileTrue(new TargetSpeakerCommand());
        // b_setShootPosition.onTrue(new InstantCommand(() -> s_Shooter.setShaftPosition(4)));
        // b_setPickupPosition.onTrue(new InstantCommand(() -> s_Shooter.setShaftPosition(6)));

        Command prepareSpeakerCommand = new ParallelCommandGroup(
            new InstantCommand(() -> s_Shooter.setAngle(0)),
            new RotateCommand(0)
        );
        b_prepareSpeaker.whileTrue(prepareSpeakerCommand);
        b_prepareSpeaker.onFalse(new InstantCommand(() -> {
            prepareSpeakerCommand.cancel();
        }));

        /* Climber */
        b_climbUp.whileTrue(new DriveClimber(1));
        b_climbUp.onFalse(( new InstantCommand(() ->{
            s_Climber.setSpeed(0);
            h_pneumatics.setClimberSolenoid(true);
        })));

        b_climbDown.whileTrue(new DriveClimber(-1));
        b_climbDown.onFalse(( new InstantCommand(() ->{ 
            s_Climber.setSpeed(0);
            h_pneumatics.setClimberSolenoid(true);
        })));
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
        NamedCommands.registerCommand("Angle to 10.7", new InstantCommand(() -> s_Shooter.setShaftRotation(10.7)));
        NamedCommands.registerCommand("Angle to 11", new InstantCommand(() -> s_Shooter.setShaftRotation(11)));
        //NamedCommands.registerCommand("command_name", new InstantCommand(() -> ___));
    }

    private void configureAutoChooser() {
        // Autonomous Sendable Chooser
        //autoChooser = new SendableChooser<Command>();
        autoChooser = AutoBuilder.buildAutoChooser("PathPlanner Test (middle note)");
        autoChooser.addOption("Main Driver (Red)", new DriverAutoMain(true));
        autoChooser.addOption("Main Driver (Blue)", new DriverAutoMain(false));
        autoChooser.addOption("Wall Side Driver (Blue)", new DriverAutoWallSide(false));
        autoChooser.addOption("Wall Side Driver (Red)", new DriverAutoWallSide(true));
        autoChooser.addOption("Zero angle", new InstantCommand(() -> s_Shooter.setShaftRotation(0)));
        autoChooser.addOption("center two piece", new DriverAutoMidTwoNote() );
        autoChooser.addOption("just shoot", new DriverAutoNoMove() );
        autoChooser.addOption("note test", new DriveToNote() );
        autoChooser.addOption("Shoot and move back", new DriverAutoMoveBack());
        autoChooser.addOption("Spit Note and move back", new SpitAndMove());

        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void stopMotors() {
        s_Shooter.setSpeed(0);
        s_Intake.setSpeed(0);
    }

    private void preparePneumatics() {
        h_pneumatics.setShooterSolenoid(false);
        h_pneumatics.setClimberSolenoid(true);
        h_pneumatics.setAmpSolenoid(false);
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
