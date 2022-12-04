package frc.team3128.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.Constants.FieldConstants;
import frc.team3128.common.swerve.SwerveModule;
import static frc.team3128.Constants.SwerveConstants.*;
import static frc.team3128.Constants.VisionConstants.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import javax.lang.model.element.ModuleElement;

public class Swerve extends SubsystemBase {
    
    public SwerveDrivePoseEstimator odometry;
    public SwerveModule[] modules;
    public WPI_Pigeon2 gyro;
    private Pose2d estimatedPose;

    private static Swerve instance;
    public boolean fieldRelative;

    public static synchronized Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    public Swerve() {
        gyro = new WPI_Pigeon2(pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();
        fieldRelative = true;

        odometry = new SwerveDrivePoseEstimator(
            getGyroRotation2d(),
            new Pose2d(),
            swerveKinematics, 
            SVR_STATE_STD,
            SVR_LOCAL_MEASUREMENT_STD,
            SVR_VISION_MEASUREMENT_STD
            );

        modules = new SwerveModule[] {
            new SwerveModule(0, Mod0.constants),
            new SwerveModule(1, Mod1.constants),
            new SwerveModule(2, Mod2.constants),
            new SwerveModule(3, Mod3.constants)
        };
    }

    /**
     * Converts inputed velocities into seperate module states that are fed to the module. Module state consists
     * the velocity and angle of a specific module.
     * 
     * @param translation Uses the x and y of a translated point inputted and interprets it as the x and y velocities
     * of the module
     * @param rotation Uses the angle inputted and unterprets it as the rotational velocity of the module
     * @param fieldRelative True when the movement of the robot is determine by the orientation of the user when the
     * robot is initialized and directions are preserved regardless of robot angle. Flase when the movement of the robot is determine by the 
     * orientation of the robot and directions are affected by the robot angle.
     */
    
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getGyroRotation2d())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
        //SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxSpeed);
        setModuleStates(moduleStates);
    }

    public void stop() {
        for (SwerveModule module : modules) {
            module.stop();
        }
    }

    public Pose2d getPose() {
        return estimatedPose;
    }

    public void resetEncoders() {
        for (SwerveModule module : modules) {
            module.resetEncoders();
        }
    }

    public void resetOdometry(Pose2d pose) { // TODO: Call this!!!!
        resetEncoders();
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

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);
        
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
        estimatedPose = odometry.getEstimatedPosition();
        Translation2d position = estimatedPose.getTranslation();
        SmartDashboard.putNumber("Robot X", position.getX());
        SmartDashboard.putNumber("Robot Y", position.getY());
        SmartDashboard.putNumber("Robot Gyro", getGyroRotation2d().getDegrees());
        SmartDashboard.putString("POSE2D",getPose().toString());
    }

    /**
     * @return rotation of the robot relative to gyro
     */
    public double getYaw() {
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
        return Rotation2d.fromDegrees(getYaw());
    }

    public Rotation2d getRotation2d() {
        return estimatedPose.getRotation();
    }

    /**
     * @return rotaiton of the robot relative to odometry
     */
    public double getHeading() {
        return getRotation2d().getDegrees();
    }
    
    public double calculateDegreesToTurn(){
        double alpha = getHeading();
        return MathUtil.inputModulus(calculateDesiredAngle() - alpha,-180,180);
    }

    public double calculateDesiredAngle(){
        Pose2d location = getPose().relativeTo(FieldConstants.HUB_POSITION);
        double theta = Math.toDegrees(Math.atan2(location.getY(),location.getX()));
        return MathUtil.inputModulus(theta - 180,-180,180);
    }
}