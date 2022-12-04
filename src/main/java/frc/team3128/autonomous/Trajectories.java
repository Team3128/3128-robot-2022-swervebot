package frc.team3128.autonomous;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Hashtable;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.team3128.Constants.SwerveConstants.*;

import frc.team3128.commands.CmdAlign;
import frc.team3128.commands.CmdInPlaceTurn;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.subsystems.Swerve;

/**
 * Store trajectories for autonomous. Edit points here. 
 * @author Daniel Wang
 */
public class Trajectories {

    private static HashMap<String, ArrayList<PathPlannerTrajectory>> trajectories = new HashMap<String, ArrayList<PathPlannerTrajectory>>();
    private static HashMap<String, Command> eventMap = new HashMap<>();

    private static SwerveAutoBuilder builder;

    public void Trajecotires(){
        initAutoSelector();
        initTrajectories();
    }

    private void initAutoSelector() {
        String[] autoStrings = new String[] {};
        NarwhalDashboard.addAutos(autoStrings);
    }

    public Command getAutonomousCommand() {
        String selectedAutoName = NarwhalDashboard.getSelectedAutoName();
        // String selectedAutoName = "3 Ball"; // uncomment and change this for testing without opening Narwhal Dashboard
        selectedAutoName = ""; //"Marriage";

        if (selectedAutoName == null) {
            return null;
        }

        return builder.fullAuto(trajectories.get(selectedAutoName));
    }

    private static void initEventMap(){
        eventMap.put("Align", new CmdAlign());
        eventMap.put("180 Turn", new CmdInPlaceTurn(180, false));
        eventMap.put("180 Turn Inturrputed", new CmdInPlaceTurn(180, true));
    }

    public static void initTrajectories() {
        final String[] trajectoryNames = {};
        for (String trajectoryName : trajectoryNames) {
            // Path path = Filesystem.getDeployDirectory().toPath().resolve("paths").resolve(trajectoryName + ".wpilib.json");
            trajectories.put(trajectoryName, PathPlanner.loadPathGroup(trajectoryName, new PathConstraints(maxSpeed, maxAcceleration)));
        }

        builder = new SwerveAutoBuilder(
            Swerve.getInstance()::getPose,
            Swerve.getInstance()::resetOdometry,
            swerveKinematics,
            new PIDConstants(translationKP,translationKI,translationKD),
            new PIDConstants(rotationKP,rotationKI,rotationKD),
            Swerve.getInstance()::setModuleStates,
            new HashMap<String,Command>(),
            Swerve.getInstance()
        );
    }

    public static PathPlannerTrajectory line(Pose2d start, Pose2d end) {
        return PathPlanner.generatePath(
            new PathConstraints(maxSpeed, maxAcceleration), 
            new PathPoint(start.getTranslation(), start.getRotation()), 
            new PathPoint(end.getTranslation(), end.getRotation())
            );
    }

    public static CommandBase lineCmd(Pose2d start, Pose2d end) {
        return builder.fullAuto(line(start, end));
    }
    
}
