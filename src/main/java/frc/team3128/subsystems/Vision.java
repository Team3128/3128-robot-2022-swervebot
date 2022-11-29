package frc.team3128.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Collection;
import java.util.HashMap;

import frc.team3128.common.hardware.camera.Camera;
import frc.team3128.common.hardware.camera.NAR_Camera;

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
        cameras.put("Shooter", new NAR_Camera(Camera.SHOOTER));
    }

    public double calculate_distance(String name) {
        NAR_Camera camera = cameras.get(name);
        return camera.getDistance();
    }

    public double getTx(String name) {
        NAR_Camera camera = cameras.get(name);
        return camera.targetYaw();
    }

    public double getTy(String name) {
        NAR_Camera camera = cameras.get(name);
        return camera.targetPitch();
    }

    public double getArea(String name) {
        NAR_Camera camera = cameras.get(name);
        return camera.targetArea();
    }

    public boolean hasValidTarget(String name) {
        NAR_Camera camera = cameras.get(name);
        return camera.hasValidTarget();
    }

    public void setLED(String name, boolean state) {
        NAR_Camera camera = cameras.get(name);
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
        for (NAR_Camera cam : getCameras()) {
            cam.update();
        }
    }
}
