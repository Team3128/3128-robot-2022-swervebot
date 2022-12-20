package frc.team3128.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Collection;
import java.util.HashMap;

import frc.team3128.Constants.FieldConstants;
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
        cameras.put(Camera.SHOOTER.hostname, new NAR_Camera(Camera.SHOOTER));
    }

    public Pose2d targetPos(String name, Pose2d robotPos) {
        return cameras.get(name).getTargetPos(robotPos);
    }

    public Pose2d robotPos(String name) {
        return cameras.get(name).getPos();
    }

    public double calculatedDistance(String name) {
        return cameras.get(name).getDistance();
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

    public Camera camSpecs(String name) {
        return cameras.get(name).camera;
    }

    public Collection<NAR_Camera> getCameras(){
        return cameras.values();
    }

    @Override
    public void periodic(){
        for (NAR_Camera cam : getCameras()) {
            cam.update();
        }
        SmartDashboard.putBoolean("HasTarget", cameras.get(Camera.SHOOTER.hostname).hasValidTarget());
        SmartDashboard.putNumber("Distance", cameras.get(Camera.SHOOTER.hostname).getDistance());
        SmartDashboard.putString("EstimatedPose",cameras.get(Camera.SHOOTER.hostname).getPos().toString());
        // if(cameras.get(Camera.SHOOTER.hostname).hasValidTarget()) {
        //     Swerve.getInstance().resetOdometry(cameras.get(Camera.SHOOTER.hostname).getPos());
        // }
        //SmartDashboard.putString("Aflack",cameras.get(Camera.SHOOTER.hostname).getPos().relativeTo(FieldConstants.HUB_POSITION).toString());
        SmartDashboard.putNumber("Geico",cameras.get(Camera.SHOOTER.hostname).targetYaw());
        SmartDashboard.putString("TARGETPOS",cameras.get(Camera.SHOOTER.hostname).getTargetPos(Swerve.getInstance().getPose()).toString());
        SmartDashboard.putString("RAWTARGET",cameras.get(Camera.SHOOTER.hostname).getTarget().toString());
    }
}
