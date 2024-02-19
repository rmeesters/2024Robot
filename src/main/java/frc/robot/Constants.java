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
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 1;

        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW- //Changed to True for testing 0129

        public static final COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
        COTSTalonFXSwerveConstants.SDS.MK4.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(22.5); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(23.6); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 2.0; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 3.0; //TODO: This must be tuned to specific robot // was 10

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 21;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(222.28); //42.28
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 22;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-100.02); //-280.02
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 13;
            public static final int canCoderID = 23; //TODO Apply to Robot
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(64.07); //244.07
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 14;
            public static final int canCoderID = 24;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-64.78); //-244.78
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class Climber {
        /* Module Specific Constants */
        /* Single falcon500 motor */
        public static final class Motor { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 31;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);

            public static final double maxSpeed = 10;
        }
    }

    public static final class Shooter {
        /* Module Specific Constants */
        /* Shooter Left motor */
        public static final class ShooterMotor { //TODO: This must be tuned to specific robot
            public static final int leftMotorID = 41;
            public static final int rightMotorID = 43;

            public static final double maxSpeed = 5;
        }
        
        /* Shooter Angle motor */
        public static final class AngleMotor { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 40;
            public static final int canCoderID = 42;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
        
            public static final double shaftAcceleration = 2000;
            public static final double shaftMaxSpeed = 2000;

            public static final double KP = 1;
            public static final double KI = 0;
            public static final double KD = 0;
        }

        public static final double canCoderLimit = -40;

        public static final double boltGrovesPerInch = 5;

        // Circumference of shaft (inches)
        public static final double shaftCircumference = 1;

        //TODO Adjust these based on robot measurements
        public static final double ArmDistanceX = 7; // Horizontal distance to bolt
        public static final double ArmDistanceY = 3; // Vertical distance to bolt
        public static final double ArmRange = 8; // Change in distance arm can make
        
        public static final double distanceFromShooterPivotToConnectionToArm = 8; // Distance of point where shooter is connected to the robot is from connection of arm to shooter
        public static final double lengthOfArm = 8; // Length of the arm
    }

    public static final class Intake {
        /* Module Specific Constants */
        /* Single falcon500 motor */
        public static final class Motor { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 51;

            public static final double maxSpeed = 5;
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class Limelight {

        public static final class Front {
            public static final String NAME = "limelight-gnomes-front";

            // Vertical angle of camera in degrees (positive means pointing up)

            // TODO Set height of camera
            public static double CAMERA_ANGLE = 0;

            // TODO Set height of camera
            public static double CAMERA_HEIGHT = 0.15;
        }

        public static final class Back {
            public static final String NAME = "limelight-gnomes-back";

            // Vertical angle of camera in degrees (positive means pointing up)

            // TODO Set height of camera
            public static double CAMERA_ANGLE = 4;

            // TODO Set height of camera
            public static double CAMERA_HEIGHT = 0.15;
        }

        public static final class Pipelines {
            public static final class Amp {
                // Meters high
                public static final double APRILTAG_HEIGHT = Units.inchesToMeters(48.0 + 1.0/8.0);

                public static final class Red {
                    // Pipelines
                    public static final int CENTER = 5;
                }

                public static final class Blue {
                    // Pipelines
                    public static final int CENTER = 6;
                }
            }

            public static final class Speaker {
                // Meters high
                public static final double APRILTAG_HEIGHT = Units.inchesToMeters(48.0 + 3.0 + 7.0/8.0);

                public static final class Red {
                    // Pipelines
                    public static final int SIDE = 3;
                    public static final int CENTER = 4;
                }

                public static final class Blue {
                    // Pipelines
                    public static final int SIDE = 8;
                    public static final int CENTER = 7;
                }
            }
            
            public static final class Source {
                // Meters high
                public static final double APRILTAG_HEIGHT = Units.inchesToMeters(48.0 + 1.0/8.0);

                public static final class Red {
                    // Pipelines
                    public static final int LEFT = 2;
                    public static final int RIGHT = 1;
                }

                public static final class Blue {
                    // Pipelines
                    public static final int LEFT = 10;
                    public static final int RIGHT = 9;
                }
            }
            
            public static final class Stage {
                // Meters high
                public static final double APRILTAG_HEIGHT = Units.inchesToMeters(36.0 + 11.0 + 1.0/2.0);

                public static final class Red {
                    // Pipelines
                    public static final int LEFT = 11;
                    public static final int RIGHT = 12;
                    public static final int CENTER = 13;
                }

                public static final class Blue {
                    // Pipelines
                    public static final int LEFT = 15;
                    public static final int RIGHT = 16;
                    public static final int CENTER = 14;
                }
            }
        }
    }

    public static final class Dimensions {
        public static final double NOTE_HEIGHT = 0.05;
    }

    public static final class Pneumatics {
        public static final int airSupplyCan = 0;
    }

}
