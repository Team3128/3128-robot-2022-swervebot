package frc.team3128.common.hardware.limelight;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
//import frc.team3128.subsystems.Swerve;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3128.subsystems.Swerve;

import static frc.team3128.Constants.VisionConstants.*;

import static frc.team3128.Constants.FieldConstants.*;

public class NAR_Camera extends PhotonCamera {

    private final Camera camera;
    
    private PhotonTrackedTarget target;

    private Swerve swerve;

    public NAR_Camera(Camera camera) {
        super(camera.hostname);
        this.camera = camera;
        swerve = Swerve.getInstance();
        setLED(false);
        setVersionCheckEnabled(false);
    }

    public void update() { 
        PhotonPipelineResult result = this.getLatestResult();
        SmartDashboard.putBoolean("HasTargets", result.hasTargets());
        if (result.hasTargets()) {
            target = result.getBestTarget();
            swerve.odometry.addVisionMeasurement(getPos(swerve.getHeading()), Timer.getFPGATimestamp() - result.getLatencyMillis()/1000);
            return;
        }
        target = null;
    }

    public double target_yaw() {
        if (!hasValidTarget()) return 0;
        return target.getYaw();
    }

    public double target_pitch() {
        if (!hasValidTarget()) return 0;
        return target.getPitch();
    }

    public double target_area() {
        if (!hasValidTarget()) return 0;
        return target.getArea();
    }

    public double target_skew() {
        if (!hasValidTarget()) return 0;
        return target.getSkew();
    }

    public Transform2d get_target() {
        if (!hasValidTarget()) return new Transform2d();
        return target.getCameraToTarget();
    }

    public List<TargetCorner> target_corners() {
        if (!hasValidTarget()) return new ArrayList<TargetCorner>();
        return target.getCorners();
    }

    public boolean hasValidTarget() {
        return target != null;
        // if (target == null) {
        //     return false;
        // }
        //return target.getArea() >= AREA_THRESHOLD;
    }

    public double get_distance() {
        if (!hasValidTarget())
            return -1;
        double ty = Units.degreesToRadians(target_pitch() + camera.cameraAngle);
        double tx = Units.degreesToRadians(target_skew());
        return (camera.targetHeight - camera.cameraHeight) / (Math.tan(ty) * Math.cos(tx)) - camera.cameraOffset;
    }

    public void setLED(boolean state){
        if(!camera.LED) return;

        if(state) {
            setLED(VisionLEDMode.kOn);
        }
        else {
            setLED(VisionLEDMode.kOff);
        }
    }
    
    public double calculateDistToGroundTarget() {
        if (!hasValidTarget())
            return -1;
        double ty = target_pitch() * Math.PI / 180;
        return (-camera.targetHeight + camera.cameraHeight) * Math.tan(ty + camera.cameraAngle);
    }

    // public Translation2d get_translation(){
    //     if (!hasValidTarget()) return new Translation2d();
    //     double distance = get_distance();
    //     double yaw = Units.degreesToRadians(target_yaw());
    //     Translation2d translation = new Translation2d(distance * Math.cos(yaw), distance * Math.sin(yaw));
    //     // Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(
    //     //                                         get_distance(),
    //     //                                         new Rotation2d(Units.degreesToRadians(target_yaw()))
    //     //                                         );
    //     return translation;
    // }

    // public Transform2d getRelativeCamPos(double gyroAngle){
    //     if(!hasValidTarget()) return new Transform2d();
    //     Transform2d transform = new Transform2d(
    //         get_translation(), 
    //         new Rotation2d(Units.degreesToRadians(gyroAngle)));
    //     // Transform2d transform = PhotonUtils.estimateCameraToTarget(
    //     //     get_translation(), 
    //     //     HUB_POSITION, 
    //     //     new Rotation2d(Units.degreesToRadians(gyroAngle))
    //     //     );
    //     return transform;
    // }

    //Only run this with the shooter limelight
    public Pose2d getPos(double gyroAngle) {
        if(!hasValidTarget()) return new Pose2d();

        double distance = get_distance() + HUB_RADIUS;
        double yaw = Units.degreesToRadians(target_yaw());
        Translation2d translation = new Translation2d(distance * Math.cos(yaw), distance * Math.sin(yaw));

        Transform2d transform = new Transform2d(translation,new Rotation2d(Units.degreesToRadians(gyroAngle)));
        Pose2d pos = HUB_POSITION.transformBy(transform.inverse());

        //Pose2d pos = PhotonUtils.estimateFieldToRobot(getRelativeCamPos(gyroAngle), HUB_POSITION, offset);
        return pos;
    }

    public Pose2d visionEstimatedPose(double gyroAngle) {
        if(!hasValidTarget()) return new Pose2d();
        double distToHubCenter = get_distance() + HUB_RADIUS;
        Rotation2d thetaHub = Rotation2d.fromDegrees(gyroAngle - target_yaw());
        Translation2d fieldPos = new Translation2d(-distToHubCenter * Math.cos(thetaHub.getRadians()), -distToHubCenter * Math.sin(thetaHub.getRadians()))
                                    .plus(HUB_POSITION.getTranslation());
        Pose2d pos = new Pose2d(fieldPos, Rotation2d.fromDegrees(gyroAngle));

        return pos;
    }

    public Pose2d visionPos(double gyroAngle) {
        if(!hasValidTarget()) return new Pose2d();
        double distToHubCenter = get_distance() + HUB_RADIUS;
        double angle = Units.degreesToRadians(gyroAngle - target_yaw() - 180);
        Translation2d fieldPos = new Translation2d(distToHubCenter * Math.cos(angle), distToHubCenter * Math.sin(angle))
                                    .plus(HUB_POSITION.getTranslation());
        Pose2d pos = new Pose2d(fieldPos, Rotation2d.fromDegrees(gyroAngle));

        return pos;
    }

    // private Pose2d getPosCamera(double gyroAngle) {
    //     if(!hasValidTarget()) return new Pose2d();
    //     Pose2d pos = HUB_POSITION.transformBy(getRelativeCamPos(gyroAngle).inverse());
    //     // Pose2d pos = PhotonUtils.estimateFieldToCamera(
    //     //     getRelativeCamPos(gyroAngle),
    //     //     HUB_POSITION);
    //     return pos;
    // }

    public void setPipeline(Pipeline pipeline) {
        setPipelineIndex(pipeline.getPipeline());
    }

    public String get_name() {
        return camera.hostname;
    }

}
