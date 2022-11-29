package frc.team3128.autonomous;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * Store trajectories for autonomous. Edit points here. 
 * @author Daniel Wang
 */
public class Trajectories {

    private static HashMap<String, ArrayList<PathPlannerTrajectory>> trajectories = new HashMap<String, ArrayList<PathPlannerTrajectory>>();

    public static void initTrajectories() {
        final String[] trajectoryNames = {};
        for (String trajectoryName : trajectoryNames) {
            // Path path = Filesystem.getDeployDirectory().toPath().resolve("paths").resolve(trajectoryName + ".wpilib.json");
            trajectories.put(trajectoryName, PathPlanner.loadPathGroup(trajectoryName, 2, 2, false));
        }
    }

    public static ArrayList<PathPlannerTrajectory> get(String name) {
        return trajectories.get(name);
    }

    public static PathPlannerTrajectory line(Pose2d pos1, Pose2d pos2) {
        return PathPlanner.generatePath(
            new PathConstraints(4, 4), 
            pos1, 
            pos2);
    }
    
}
