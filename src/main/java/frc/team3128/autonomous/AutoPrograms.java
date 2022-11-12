package frc.team3128.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.Swerve;

/**
 * Class to store information about autonomous routines.
 * @author Daniel Wang, Mason Lam
 */

public class AutoPrograms {

    private static Swerve swerve;

    public AutoPrograms() {
        swerve = Swerve.getInstance();

        initAutoSelector();
    }

    private void initAutoSelector() {
        String[] autoStrings = new String[] {};
        NarwhalDashboard.addAutos(autoStrings);
    }

    public Command getAutonomousCommand() {
        String selectedAutoName = NarwhalDashboard.getSelectedAutoName();
        // String selectedAutoName = "3 Ball"; // uncomment and change this for testing without opening Narwhal Dashboard
        selectedAutoName = "3Ball"; //"Marriage";

        if (selectedAutoName == null) {
            return null;
        }

        Pose2d initialPose = null;
        Command autoCommand = null;

        switch (selectedAutoName) {
            case("Marriage"):
                initialPose = Trajectories.get("Marriage").getInitialPose();
                autoCommand = Trajectories.path("Marriage");
                break;
            case("3Ball"):
                initialPose = Trajectories.get("3Ball").getInitialPose();
                autoCommand = Trajectories.path("3Ball");
                break;    
            default: 
                Log.info("Auto Selector", "Something went wrong in getting the auto name - misspelling?");
                break;
        }

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
