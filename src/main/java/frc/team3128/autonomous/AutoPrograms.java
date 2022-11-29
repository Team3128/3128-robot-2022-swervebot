package frc.team3128.autonomous;

import java.util.HashMap;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.Constants.SwerveConstants;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.Swerve;

/**
 * Class to store information about autonomous routines.
 * @author Daniel Wang, Mason Lam
 */

public class AutoPrograms {

    private static SwerveAutoBuilder builder;

    public static Swerve swerve;

    public AutoPrograms() {
        Trajectories.initTrajectories();
        swerve = Swerve.getInstance();
        initAutoSelector();
        builder = new SwerveAutoBuilder(
            swerve::getPose,
            swerve::resetOdometry,
            SwerveConstants.swerveKinematics,
            new PIDConstants(0,0,0),
            new PIDConstants(0,0,0),
            swerve::setModuleStates,
            new HashMap<String,Command>(),
            swerve
        );
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

        Pose2d initialPose = null;
        Command autoCommand = null;

        // switch (selectedAutoName) {
        //     case("Marriage"):
        //         initialPose = Trajectories.get("Marriage").getInitialPose();
        //         autoCommand = Trajectories.path("Marriage");
        //         break;
        //     case("3Ball"):
        //         initialPose = Trajectories.get("3Ball").getInitialPose();
        //         autoCommand = Trajectories.path("3Ball");
        //         break; 
        //     case("Forward"):
        //         initialPose = Trajectories.get("Forward").getInitialPose();
        //         autoCommand = Trajectories.path("Forward");
        //         break;
        //     case("180Turn"):
        //         initialPose = Trajectories.get("Forward").getInitialPose();
        //         autoCommand = new CmdInPlaceTurn(180);
        //     default: 
        //         Log.info("Auto Selector", "Something went wrong in getting the auto name - misspelling?");
        //         break;
        // }

        swerve.resetOdometry(initialPose);
        return autoCommand;
    }
    
    // /** 
    //  * Follow trajectory and intake balls along the path
    //  */
    // private SequentialCommandGroup IntakePathCmd(String trajectory) {
    //     ParallelDeadlineGroup movement = new ParallelDeadlineGroup(
    //                                         trajectoryCmd(trajectory), 
    //                                         new ScheduleCommand(new CmdExtendIntakeAndRun()));
    //     return new SequentialCommandGroup(new InstantCommand(intake::ejectIntake, intake), movement);
    // }

    /**
     * Flip 180 degrees rotation wise but keep same pose translation 
     */
    private Pose2d inverseRotation(Pose2d pose) {
        return new Pose2d(pose.getTranslation(), new Rotation2d(pose.getRotation().getRadians() + Math.PI));
    }
}
