package frc.team3128.autonomous;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.team3128.Constants.SwerveConstants;
import frc.team3128.subsystems.Swerve;


/**
 * Store trajectories for autonomous. Edit points here. 
 * @author Daniel Wang
 */
public class Trajectories {

    //private static HashMap<String, Trajectory> trajectories = new HashMap<String, Trajectory>();

    public static PIDController PID1 = new PIDController(1, 0, 0);
    public static PIDController PID2 = new PIDController(1, 0, 0);
    public static PIDController PID3 = new PIDController(0.65, 0, 0.1);

    private static HashMap<String, PathPlannerTrajectory> trajectories = new HashMap<String,PathPlannerTrajectory>();

    static{
        final String[] trajectoryNames = {"Marriage", "3Ball", "Forward"};
        for (String trajectoryName : trajectoryNames) {
            trajectories.put(trajectoryName, PathPlanner.loadPath(trajectoryName, SwerveConstants.maxSpeed, SwerveConstants.maxAcceleration));
            //Path path = Filesystem.getDeployDirectory().toPath().resolve("paths").resolve(trajectoryName + ".wpilib.json");
            // try {
            //     trajectories.put(trajectoryName, TrajectoryUtil.fromPathweaverJson(path));
            // } catch (IOException ex) {
            //     DriverStation.reportError("IOException loading trajectory " + trajectoryName, true);
            // }
        }
        var tab = Shuffleboard.getTab("test");
        tab.add(PID1);
        tab.add(PID2);
        tab.add(PID3);
    }

    public static PPSwerveControllerCommand path(String name) {
        Swerve swerve = Swerve.getInstance();
        return new PPSwerveControllerCommand(
            trajectories.get(name), 
            Swerve.getInstance()::getPose, 
            SwerveConstants.swerveKinematics, 
            PID1, 
            PID2, 
            PID3, 
            outputModuleStates -> swerve.setStates(outputModuleStates),  
            swerve);
    }

    public static Trajectory get(String name) {
        return trajectories.get(name);
    }
    
}
