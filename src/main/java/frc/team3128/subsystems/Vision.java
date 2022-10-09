package frc.team3128.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Collection;
import java.util.HashMap;

import frc.team3128.common.hardware.limelight.Camera;
import frc.team3128.common.hardware.limelight.NAR_Camera;

import static frc.team3128.Constants.FieldConstants.*;

public class Vision extends SubsystemBase{
    private static Vision instance;

    private HashMap<String,NAR_Camera> cameras;

    public static synchronized Vision getInstance(){
        if(instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    public Vision() {
        NAR_Camera shooter_limelight = new NAR_Camera(Camera.SHOOTER);
        cameras.put("Shooter", shooter_limelight);
    }

    public double calculate_distance(String name) {
        NAR_Camera camera = cameras.get(name);
        if (camera == null) return 0;
        return camera.get_distance();
    }

    public double getTx(String name) {
        NAR_Camera camera = cameras.get(name);
        if (camera == null) return 0;
        return camera.target_yaw();
    }

    public double getTy(String name) {
        NAR_Camera camera = cameras.get(name);
        if (camera == null) return 0;
        return camera.target_pitch();
    }

    public double getArea(String name) {
        NAR_Camera camera = cameras.get(name);
        if (camera == null) return 0;
        return camera.target_area();
    }

    public boolean hasValidTarget(String name) {
        NAR_Camera camera = cameras.get(name);
        if (camera == null) return false;
        return camera.hasValidTarget();
    }

    public void setLED(String name, boolean state) {
        NAR_Camera camera = cameras.get(name);
        if (camera == null) return;
        camera.setLED(state);
    }

    public NAR_Camera getCamera(String name) {
        return cameras.get(name);
    }

    public Collection<NAR_Camera> getCameras(){
        return cameras.values();
    }

    public Pose2d visionEstimatedPose(double gyroAngle) {
        /*DO NOT USE UNLESS LIMELIGHT HAS A VALID TARGET */
        double distToHubCenter = cameras.get("Shooter").get_distance() + HUB_RADIUS;
        Rotation2d thetaHub = Rotation2d.fromDegrees(gyroAngle - cameras.get("Shooter").target_yaw());
        Translation2d fieldPos = new Translation2d(-distToHubCenter * Math.cos(thetaHub.getRadians()), -distToHubCenter * Math.sin(thetaHub.getRadians()))
                                    .plus(HUB_POSITION.getTranslation());
        return new Pose2d(fieldPos, Rotation2d.fromDegrees(gyroAngle));
    }
    
    @Override
    public void periodic(){
        for (NAR_Camera cam : getCameras()) {
            cam.update();
        }
    }
}
