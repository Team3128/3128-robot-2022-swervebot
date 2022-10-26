package frc.team3128.autonomous;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.team3128.Constants.SwerveConstants;


/**
 * Store trajectories for autonomous. Edit points here. 
 * @author Daniel Wang
 */
public class Trajectories {

    //private static HashMap<String, Trajectory> trajectories = new HashMap<String, Trajectory>();

    private static HashMap<String, PathPlannerTrajectory> trajectories = new HashMap<String,PathPlannerTrajectory>();

    static{
        final String[] trajectoryNames = {};
        for (String trajectoryName : trajectoryNames) {
            trajectories.put(trajectoryName, PathPlanner.loadPath(trajectoryName, SwerveConstants.maxSpeed, SwerveConstants.maxAcceleration));
            //Path path = Filesystem.getDeployDirectory().toPath().resolve("paths").resolve(trajectoryName + ".wpilib.json");
            // try {
            //     trajectories.put(trajectoryName, TrajectoryUtil.fromPathweaverJson(path));
            // } catch (IOException ex) {
            //     DriverStation.reportError("IOException loading trajectory " + trajectoryName, true);
            // }
        }
    }

    public static PPSwerveControllerCommand path(String name) {
        return null;
    }

    public static Trajectory get(String name) {
        return trajectories.get(name);
    }
    
}
