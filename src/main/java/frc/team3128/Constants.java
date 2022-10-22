package frc.team3128;

import static frc.team3128.common.hardware.motorcontroller.MotorControllerConstants.FALCON_ENCODER_RESOLUTION;
import static frc.team3128.common.hardware.motorcontroller.MotorControllerConstants.SPARKMAX_ENCODER_RESOLUTION;

public class Constants {

    public static class ConversionConstants {

        public static final double SPARK_VELOCITY_FACTOR = SPARKMAX_ENCODER_RESOLUTION / 60; // RPM to nu/s
        public static final double FALCON_NUp100MS_TO_RPM = 10 * 60 / FALCON_ENCODER_RESOLUTION; // sensor units per 100 ms to rpm
        public static final double FALCON_NUpS_TO_RPM = 60 / FALCON_ENCODER_RESOLUTION; // sensor units per second to rpm
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
        public static final int HOPPER_MOTOR_1_ID = 1;
        public static final int HOPPER_MOTOR_2_ID  = 2;
        public static final int HOPPER_MOTOR_OUTTAKE_ID  = 3;
        public static final int HOPPER_MOTOR_SERIALIZER_ID  = 4;
        public static final int HOPPER_MOTOR_SHOOT_ID = 5;

        public static final int HOPPER_MOTOR_POWER = 1;
        public static final int HOPPER_SERIALIZER_POWER = 1;
        public static final int HOPPER_OUTTAKE_POWER = 1;
        public static final int HOPPER_SHOOT_POWER = 1;

        public static final double COLOR_SENSOR_TOLERANCE = 1.5;


    }

    public static class IntakeConstants {

    }

    public static class ShooterConstants {
        public static final double kF = 0;
        
        public static final double LOW_kP = 0;
        public static final double LOW_kI = 0;
        public static final double LOW_kD = 0;
        public static final double LOW_kS = 0;
        public static final double LOW_kV = 0;
        public static final double LOW_kA = 0;

        public static final double HIGH_kP = 0;
        public static final double HIGH_kI = 0;
        public static final double HIGH_kD = 0;
        public static final double HIGH_kS = 0;
        public static final double HIGH_kV = 0;
        public static final double HIGH_kA = 0;

        public static final int LEFT_SHOOTER_ID = 0;
        public static final int RIGHT_SHOOTER_ID = 0;

        public static final double RPM_THRESHOLD_PRECENT = 0.05;
        public static final double PLATEAU_THRESHOLD = 1;
        public static final double RPM_THRESHOLD_PRECENT_MAX = 0.15;
        public static final double TIME_TO_MAX_THRESHOLD = 8;
        public static final double PLATEUA_COUNT = 6;

    }
        
    public static class HoodConstants {

    }
    
    public static class VisionConstants {

        public static final String SHOOTER_HOSTNAME = "limelight-cog";

        public static final double HUB_HEIGHT = 104;

        public static final double BALL_TARGET_HEIGHT = 9.5 / 2;
    }
}
