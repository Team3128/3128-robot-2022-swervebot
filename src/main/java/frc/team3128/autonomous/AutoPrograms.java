package frc.team3128.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.common.utility.Log;

/**
 * Class to store information about autonomous routines.
 * @author Daniel Wang, Mason Lam
 */

public class AutoPrograms {

    public AutoPrograms() {

        initAutoSelector();
    }

    private void initAutoSelector() {
        String[] autoStrings = new String[] {};
        NarwhalDashboard.addAutos(autoStrings);
    }

    public Command getAutonomousCommand() {
        String selectedAutoName = NarwhalDashboard.getSelectedAutoName();
        // String selectedAutoName = "3 Ball"; // uncomment and change this for testing without opening Narwhal Dashboard

        if (selectedAutoName == null) {
            return null;
        }

        Pose2d initialPose = null;
        Command autoCommand = null;

        switch (selectedAutoName) {
            default: 
                Log.info("Auto Selector", "Something went wrong in getting the auto name - misspelling?");
                break;
        }

        // drive.resetPose(initialPose);
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
        return new Pose2d(pose.getTranslation(), pose.getRotation().unaryMinus());
    }
}
