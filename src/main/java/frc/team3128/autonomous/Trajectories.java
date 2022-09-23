package frc.team3128.autonomous;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * Store trajectories for autonomous. Edit points here. 
 * @author Daniel Wang
 */
public class Trajectories {

    private static HashMap<String, Trajectory> trajectories = new HashMap<String, Trajectory>();

    static{
        final String[] trajectoryNames = {};
        for (String trajectoryName : trajectoryNames) {
            Path path = Filesystem.getDeployDirectory().toPath().resolve("paths").resolve(trajectoryName + ".wpilib.json");
            try {
                trajectories.put(trajectoryName, TrajectoryUtil.fromPathweaverJson(path));
            } catch (IOException ex) {
                DriverStation.reportError("IOException loading trajectory " + trajectoryName, true);
            }
        }
    }

    public static Trajectory get(String name) {
        return trajectories.get(name);
    }
    
}
