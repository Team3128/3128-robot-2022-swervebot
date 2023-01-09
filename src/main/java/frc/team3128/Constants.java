package frc.team3128;

import static frc.team3128.common.hardware.motorcontroller.MotorControllerConstants.FALCON_ENCODER_RESOLUTION;
import static frc.team3128.common.hardware.motorcontroller.MotorControllerConstants.SPARKMAX_ENCODER_RESOLUTION;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.team3128.common.swerve.SecondOrderSwerveDriveKinematics;
import frc.team3128.common.swerve.SwerveModuleConstants;

public class Constants {

    public static class ConversionConstants {

        public static final double SPARK_VELOCITY_FACTOR = SPARKMAX_ENCODER_RESOLUTION / 60; // RPM to nu/s
        public static final double FALCON_NUp100MS_TO_RPM = 10 * 60 / FALCON_ENCODER_RESOLUTION; // sensor units per 100 ms to rpm
        public static final double FALCON_NUpS_TO_RPM = 60 / FALCON_ENCODER_RESOLUTION; // sensor units per second to rpm
    }

    public static class SwerveConstants {
        public static final int pigeonID = 30; 
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(20.75); // Units.inchesToMeters(21.73);
        public static final double wheelBase = Units.inchesToMeters(20.75); // Units.inchesToMeters(21.73);
        public static final double wheelDiameter = Units.inchesToMeters(4); // Units.inchesToMeters(3.94);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = 6.75; // (6.86 / 1.0); //6.86:1
        public static final double angleGearRatio = 21.43; // (12.8 / 1.0); //12.8:1

        public static final SecondOrderSwerveDriveKinematics swerveKinematics = new SecondOrderSwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0 + Units.inchesToMeters(0.075), -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        // switched 364 pid values to SDS pid values
        public static final double angleKP = 0.3; // 0.6; // citrus: 0.3
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0; // 12.0; // citrus: 0
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        // need to characterize this
        public static final double driveKP = 0.05;//1.476;//0.05; // citrus: 0.05 //sysid 2.9424
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        // TODO: need to characterize
        public static final double driveKS = (0.60094/12);//(0.49321 / 12);
        public static final double driveKV = (1.1559/12);//(2.4466 / 12);
        public static final double driveKA = (0.12348/12);//(0.22036 / 12);

        /* Angle Motor Characterization Values */
        public static final double angleKS = (0.60094/12);//(0.49321 / 12);
        public static final double angleKV = (1.1559/12);//(2.4466 / 12);
        public static final double angleKA = (0.12348/12);//(0.22036 / 12);

        /*Chasis Turing PID Values*/
        public static final double turnKP = 0.65;
        public static final double turnKI = 0;
        public static final double turnKD = 0;
        public static final double turnKF = 0;

        public static final double turnTolerance = 2;

        /* Swerve Profiling Values */
        public static final double maxSpeed = 3; //4.5// 4.96824; // citrus: 4.5 //meters per second - 16.3 ft/sec
        public static final double maxAcceleration = 2;
        public static final double maxAngularVelocity = 1;//3; //11.5; // citrus: 10

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = true;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        // TODO: Figure out angle offsets
        public static final class Mod0 {
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 20;
            public static final double angleOffset = -157.763671875; // -156.357421875;//-46.5 + 90; //104.5;//19.599609375; // 19.51171875;//-51.85546875; // 37.35; // degrees
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 21;
            public static final double angleOffset = 129.375; //126.38671875000001; //23.466 + 90;//-132.25;//311.66015625 - 360; //132.5390625; //311.8359375; //10.45; // degrees
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 22;
            public static final double angleOffset = -69.697265625; //-72.0703125;//-70.751953125; //-70.75; //109.51171875; //38.75; // degrees
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 23;
            public static final double angleOffset = -54.31640625; //-52.91015625; //-52.9; //306.2109375; //307.6171875; // 58.88; // degrees
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

        public static final String SHOOTER_HOSTNAME = "limelight-cog";

        public static final double HUB_HEIGHT = 104;

        public static final double BALL_TARGET_HEIGHT = 9.5 / 2;
    }
}
