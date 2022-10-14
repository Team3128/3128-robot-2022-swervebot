package frc.team3128.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.common.swerve.SwerveModule;
import static frc.team3128.Constants.SwerveConstants.*;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry odometry;
    public SwerveModule[] modules;
    public static WPI_Pigeon2 gyro;
    private static Swerve instance;

    public Swerve() {
        gyro = new WPI_Pigeon2(0);
        zeroGyro();

        odometry = new SwerveDriveOdometry(swerveKinematics, getGyroRotation2d());

        modules = new SwerveModule[] {
            new SwerveModule(0, Mod0.constants),
            new SwerveModule(1, Mod1.constants),
            new SwerveModule(2, Mod2.constants),
            new SwerveModule(3, Mod3.constants)
        };
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getGyroRotation2d())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxSpeed);

        for (SwerveModule module : modules) {
            module.setDesiredState(moduleStates[module.moduleNumber]);
        }
    }

    public void stop() {
        for (SwerveModule module : modules) {
            module.stop();
        }
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) { // TODO: Call this!!!!
        odometry.resetPosition(pose, getGyroRotation2d());
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule module : modules) {
            states[module.moduleNumber] = module.getState();
        }
        return states;
    }

    @Override
    public void periodic() {
        odometry.update(getGyroRotation2d(), getStates());
        for(SwerveModule module : modules){
            SmartDashboard.putNumber("Mod " + module.moduleNumber + " Cancoder", module.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + module.moduleNumber + " Integrated", module.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + module.moduleNumber + " Velocity", module.getState().speedMetersPerSecond);    
        }
        Pose2d pose = odometry.getPoseMeters();
        Translation2d position = pose.getTranslation();
        SmartDashboard.putNumber("Robot X", position.getX());
        SmartDashboard.putNumber("Robot Y", position.getY());
        SmartDashboard.putNumber("Robot Gyro", getGyroRotation2d().getRadians());
    }

    public double getHeading() {
        return -gyro.getAngle();
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public void zeroGyro() {
        gyro.reset();
    }

    public Rotation2d getGyroRotation2d() {
        return gyro.getRotation2d();
    }
    
    public static synchronized Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }
    
}