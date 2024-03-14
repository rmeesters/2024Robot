package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double STICK_DEAD_BAND = 0.1;

    public static final class Swerve {

        //public static final int pigeonID = 1;

        // Always ensure Gyro is CCW+ CW-
        public static final boolean INVERT_GYRO = false;

        public static final COTSTalonFXSwerveConstants chosenModule =
                COTSTalonFXSwerveConstants.SDS.MK4.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4.driveRatios.L2);

        /* Drivetrain Constants */
        // Distance from left wheel to right
        public static final double TRACK_WIDTH = Units.inchesToMeters(25);
        // Distance from front wheel to back
        public static final double WHEEL_BASE = Units.inchesToMeters(22.25);
        public static final double WHEEL_CIRCUMFERENCE = chosenModule.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = chosenModule.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue DRIVE_MOTOR_INVERT = chosenModule.driveMotorInvert;
        public static final InvertedValue ANGLE_MOTOR_INVERT = chosenModule.angleMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue CANCODER_INVERT = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int ANGLE_CURRENT_LIMIT = 25;
        public static final int ANGLE_CURRENT_THRESHOLD = 40;
        public static final double ANGLE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int driveCurrentLimit = 35;
        public static final int DIVE_CURRENT_THRESHOLD = 60;
        public static final double DRIVE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Angle Motor PID Values */
        public static final double ANGLE_KP = chosenModule.angleKP;
        public static final double ANGLE_KI = chosenModule.angleKI;
        public static final double ANGLE_KD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double DRIVE_KP = 1.1;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double DRIVE_KS = 0.32;
        public static final double DRIVE_KV = 1.51;
        public static final double DRIVE_KA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = 4.5;
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = 3.0;

        /* Neutral Modes */
        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 1;
            public static final int ANGLE_MOTOR_ID = 11;
            public static final int CANCODER_ID = 21;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(333.89);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(
                DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 2;
            public static final int ANGLE_MOTOR_ID = 12;
            public static final int CANCODER_ID = 22;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(40.78);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(
                DRIVE_MOTOR_ID, ANGLE_MOTOR_ID,
                    CANCODER_ID, ANGLE_OFFSET);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 3;
            public static final int ANGLE_MOTOR_ID = 13;
            public static final int CANCODER_ID = 23;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(-355.25);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(
                DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 4;
            public static final int ANGLE_MOTOR_ID = 14;
            public static final int CANCODER_ID = 24;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(-63.11);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(
                DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }
    }

    public static final class Climber {

        // Highest and lowest height
        public static final double MIN = 0;
        public static final double MAX = 100;
        
        // Time between moving the climber and activating/deactivating blocker
        public static final double DELAY_AFTER_PNEUMATICS = 0.1;

        /* Module Specific Constants */
        /* Single falcon500 motor */
        public static final class BeltMotor {
            public static final int MOTOR_ID = 31;
        }
    }

    public static final class Shooter {

        //public static final double SHOOT_DELAY = 0.5;
        public static final double TARGET_SHOOTER_SPEED = 98;

        public static final double IDEAL_INTAKE_POSITION_IN_INCHES = 3.0;

        public static final double CANCODER_MIN = -30;
        public static final double CANCODER_MAX = 11.7;

        public static final double SHAFT_LENGTH_IN_INCHES = 8;
        public static final double GROOVES_PER_INCH = 5;

        /* Module Specific Constants */
        /* Shooter Left motor */
        public static final class WheelMotor {
            public static final int LEFT_MOTOR_ID = 41;
            public static final int RIGHT_MOTOR_ID = 43;
        }

        /* Shooter Angle motor */
        public static final class AngleMotor {
            public static final int MOTOR_ID = 40;
            public static final int CANCODER_ID = 42;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0);

            public static final double ACCELERATION = 9999;
            public static final double MAX_SPEED = 9999;

            public static final double KP = 30;
            public static final double KI = 0;
            public static final double KD = 0;
        }
    }

    public static final class Intake {
        /* Module Specific Constants */
        /* Single falcon500 motor */
        public static final class ConveyerMotor {
            public static final int MOTOR_ID = 51;
        }
    }

    public static final class Autos {
        public static final double MAX_SPEED_IN_METERS_PER_SECOND = 3;
        public static final double MAX_ACCELERATION_IN_METERS_PER_SECOND_SQUARED = 3;
        public static final double MAX_ANGULAR_SPEED_IN_RADIANS_PER_SECOND = Math.PI;
        public static final double MAX_ANGULAR_ACCELERATION_IN_RADIANS_PER_SECOND_SQUARED = Math.PI;

        public static final double CONTROLLER_PX = 4;
        public static final double CONTROLLER_PY = 4;
        public static final double CONTROLLER_ANGLE = 2;

        /* For a more smooth rotation */
        public static final double LARGEST_POSSIBLE_ROTATION = 25;

        /* Target Speaker Command */
        public static final double TARGET_SPEAKER_SHOOTER_ANGLE_VERTICAL_OFFSET_IN_METERS = 0.6;
        public static final double HORIZONTAL_DISTANCE_FROM_LIMELIGHT_TO_SHOOTER_ANGLE_PIVOT_IN_METERS = 0.56;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_IN_RADIANS_PER_SECOND, MAX_ANGULAR_ACCELERATION_IN_RADIANS_PER_SECOND_SQUARED);
    }

    public static final class Limelight {

        public static final class Front {
            public static final String NAME = "limelight-gnomesf";

            public static double ANGLE = -1;
            public static double HEIGHT = 0.25;
        }

        public static final class Back {
            public static final String NAME = "limelight-gnomesb";

            public static double ANGLE = 8;
            public static double HEIGHT = 0.235;
        }

        public static final class Pipelines {
            public static final int SPEAKER = 0;
        }
    }

    public static final class Pneumatics {
        public static final int CLIMBER_ID = 15;
        public static final int SHOOTER_ID = 12;
        public static final int PUSHER_ID = 13;

        public static final double MIN_PRESSURE = 70;
        public static final double MAX_PRESSURE = 120;
    }

    public static final class Map {
        public static final double NOTE_HEIGHT = 0.05;

        public static final class Amp {
            public static final double APRILTAG_HEIGHT = Units.inchesToMeters(48 + 1.0 / 8 + 3);
        }

        public static final class Speaker {
            public static final double APRILTAG_HEIGHT = Units.inchesToMeters(53 + 7.0 / 8 + 3);
        }

        public static final class Source {
            public static final double APRILTAG_HEIGHT = Units.inchesToMeters(48 + 1.0 / 8 + 3);
        }

        public static final class Stage {
            public static final double APRILTAG_HEIGHT = Units.inchesToMeters(47 + 1.0 / 2 + 3);
        }
    }
}
