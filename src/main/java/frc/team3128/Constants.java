package frc.team3128;

import static frc.team3128.common.hardware.motorcontroller.MotorControllerConstants.FALCON_ENCODER_RESOLUTION;
import static frc.team3128.common.hardware.motorcontroller.MotorControllerConstants.SPARKMAX_ENCODER_RESOLUTION;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.team3128.common.swerve.SwerveModuleConstants;
import frc.team3128.common.utility.interpolation.InterpolatingDouble;
import frc.team3128.common.utility.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static class ConversionConstants {

        public static final double SPARK_VELOCITY_FACTOR = SPARKMAX_ENCODER_RESOLUTION / 60; // RPM to nu/s
        public static final double FALCON_NUp100MS_TO_RPM = 10 * 60 / FALCON_ENCODER_RESOLUTION; // sensor units per 100 ms to rpm
        public static final double FALCON_NUpS_TO_RPM = 60 / FALCON_ENCODER_RESOLUTION; // sensor units per second to rpm
    }

    public static class SwerveConstants {
        public static final int pigeonID = 0; // TODO
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = 0; // TODO // Units.inchesToMeters(21.73);
        public static final double wheelBase = 0; // TODO // Units.inchesToMeters(21.73);
        public static final double wheelDiameter = 0; // TODO // Units.inchesToMeters(3.94);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = 6.75; // (6.86 / 1.0); //6.86:1
        public static final double angleGearRatio = 150/7; // (12.8 / 1.0); //12.8:1

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        public static final double TURN_TOLERANCE = 3;

        /* Turning PID Values */
        public static final double turnKP = 0.1;
        public static final double turnKI = 0;
        public static final double turnKD = 0;
        public static final double turnKF = 0.1;

        /* Angle Motor PID Values */
        // switched 364 pid values to SDS pid values
        public static final double angleKP = 0.2; // 0.6;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.1; // 12.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        // need to characterize this
        public static final double driveKP = 0.10;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        // TODO: need to characterize
        public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.96824; //meters per second - 16.3 ft/sec
        public static final double maxAngularVelocity = 11.5;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        // TODO: Figure out angle offsets
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final double angleOffset = 0; // 37.35; // degrees
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final double angleOffset = 0; //10.45; // degrees
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final double angleOffset = 0; //38.75; // degrees
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final double angleOffset = 0; // 58.88; // degrees
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static class DriveConstants {

        // public static final double ENCODER_DISTANCE_PER_MARK = WHEEL_RADIUS_METERS * 2 * Math.PI / FALCON_ENCODER_RESOLUTION;
        // public static final double DRIVE_NU_TO_METER = ENCODER_DISTANCE_PER_MARK / DRIVE_GEARING; // meters driven per encoder tick
        // public static final double DRIVE_NUp100MS_TO_MPS = DRIVE_NU_TO_METER * 10; // sensor units per 100 ms to m/s of drivetrain
        // public static final double MAX_DRIVE_VEL_NUp100MS = 6380 * FALCON_ENCODER_RESOLUTION / 60 / 10; // max angular velocity of drivetrain (encoder, not wheel) in sensor units per 100 ms - 6380 RPM * RESOLUTION nu/rot * 1 min/60s * 1s/(10*100ms)

    }

    public static class ClimberConstants {

    }

    public static class HopperConstants {

    }

    public static class IntakeConstants {

    }

    public static class ShooterConstants {

    }

    public static class HoodConstants {

    }

    public static class VisionConstants {

        public static final double SCREEN_WIDTH = 320;
        public static final double SCREEN_HEIGHT = 240;
    
        public static final double HORIZONTAL_FOV = 59.6; //degrees
        public static final double VERTICAL_FOV = 45.7; //degrees

        public static final double TX_THRESHOLD = 3;

        public static final double AREA_THRESHOLD = 0;

        public static final double TARGET_AREA = 6.25 * 6.25; //inches

        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> visionMap = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();

        public static final Matrix<N3,N1> SVR_STATE_STD = VecBuilder.fill(0.05,0.05,Units.degreesToRadians(0));
 
        public static final Matrix<N1,N1> SVR_LOCAL_MEASUREMENT_STD = VecBuilder.fill(Units.degreesToRadians(0));
 
        public static final Matrix<N3,N1> SVR_VISION_MEASUREMENT_STD = VecBuilder.fill(0.5,0.5,Units.degreesToRadians(0));

        static {
            visionMap.put(new InterpolatingDouble(3.34),new InterpolatingDouble(4.0));
        }
    }
    
    public static class FieldConstants{
        public static final Pose2d HUB_POSITION = new Pose2d(Units.inchesToMeters(324), Units.inchesToMeters(162),new Rotation2d(0));
        public static final double FIELD_X_LENGTH = Units.inchesToMeters(648); // meters
        public static final double FIELD_Y_LENGTH = Units.inchesToMeters(324); // meters
        public static final double HUB_RADIUS = Units.inchesToMeters(26.69); // meters
    }
}
