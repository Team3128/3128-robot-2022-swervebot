package frc.team3128.subsystems;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        cameras = new HashMap<String,NAR_Camera>();
        NAR_Camera shooter_limelight = new NAR_Camera(Camera.SHOOTER);
        Shuffleboard.getTab("Test").add(new HttpCamera("limelight-sog", "http://10.31.28.25:5800/stream.mjpg"));
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
    
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Area",cameras.get("Shooter").target_area());
        SmartDashboard.putBoolean("ValidTarget",cameras.get("Shooter").hasValidTarget());
        for (NAR_Camera cam : getCameras()) {
            cam.update();
        }
    }
}
