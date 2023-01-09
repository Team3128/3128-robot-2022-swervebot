package frc.team3128.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.common.swerve.SecondOrderChassisSpeeds;
import frc.team3128.common.swerve.SecondOrderSwerveDriveKinematics;
import frc.team3128.common.swerve.SecondOrderSwerveDriveOdometry;
import frc.team3128.common.swerve.SecondOrderSwerveModuleState;
import frc.team3128.common.swerve.SwerveModule;
import static frc.team3128.Constants.SwerveConstants.*;

import javax.lang.model.element.ModuleElement;

public class Swerve extends SubsystemBase {
    public SecondOrderSwerveDriveOdometry odometry;
    public SwerveModule[] modules;
    public static WPI_Pigeon2 gyro;
    private static Swerve instance;
    public boolean fieldRelative;

    private Translation2d previousTranslation;
    private long previousTime;
    private double previousRotation;

    public Swerve() {
        gyro = new WPI_Pigeon2(pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();
        fieldRelative = true;

        odometry = new SecondOrderSwerveDriveOdometry(swerveKinematics, getGyroRotation2d());

        modules = new SwerveModule[] {
            new SwerveModule(0, Mod0.constants),
            new SwerveModule(1, Mod1.constants),
            new SwerveModule(2, Mod2.constants),
            new SwerveModule(3, Mod3.constants)
        };
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {

        if(previousTranslation == null) {
            previousTranslation = translation;
            previousTime = WPIUtilJNI.now();
            previousRotation = rotation;
        }

        double dt = (WPIUtilJNI.now() - previousTime) / 1000000000.0;

        //Get SecondOrderChassisSpeeds
        SecondOrderChassisSpeeds secondOrderChassisSpeeds = new SecondOrderChassisSpeeds(
            translation.getX(),
            translation.getY(),
            rotation,
            (translation.getX() - previousTranslation.getX()) / dt,
            (translation.getY() - previousTranslation.getY()) / dt,
            rotation-previousRotation/dt //TODO: I'm not sure if rotation is an angular velocity
        );
        
        SecondOrderSwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(
            fieldRelative ? SecondOrderChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, secondOrderChassisSpeeds.axMetersPerSecondSq,
                secondOrderChassisSpeeds.ayMetersPerSecondSq, secondOrderChassisSpeeds.alphaRadiansPerSecondSq, getGyroRotation2d())
                : secondOrderChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxSpeed);

        setStates(moduleStates);
    }

    public void setStates(SecondOrderSwerveModuleState[] states) {
        for (SwerveModule module : modules) {
            module.setDesiredState(states[module.moduleNumber]);
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
        zeroGyro(pose.getRotation().getDegrees());
        odometry.resetPosition(pose, getGyroRotation2d());
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule module : modules) {
            states[module.moduleNumber] = module.getState();
        }
        return states;
    }
    
    public void toggle() {
        if (fieldRelative) {
            fieldRelative = false;
            return;
        }
        fieldRelative = true;
    }

    public void setModuleStates(SecondOrderSwerveModuleState[] desiredStates) {
        SecondOrderSwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);
        
        for (SwerveModule module : modules){
            module.setDesiredState(desiredStates[module.moduleNumber]);
        }
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
        SmartDashboard.putNumber("Robot Gyro", getGyroRotation2d().getDegrees());
        SmartDashboard.putString("POSE2D",getPose().toString());
    }

    public double getHeading() {
        return gyro.getYaw();
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public void zeroGyro() {
        gyro.reset();
    }

    public void zeroGyro(double reset) {
        gyro.setYaw(reset);
    }


    public Rotation2d getGyroRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }
    
    public static synchronized Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }
    
}