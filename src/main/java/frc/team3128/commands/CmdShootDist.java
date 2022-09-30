package frc.team3128.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystem.Shooter;

public class CmdShootDist extends CommandBase{

    private Shooter shooter;

    public CmdShootDist() {
        shooter = Shooter.getInstance();
        // Need other subsystems
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // Would initialize limelight/photon vision
    }

    @Override 
    public void execute() {
        // Need limelight/photon vision for distance
        shooter.beginShoot(0); // Need distance for RPM
        // No hood subsystem for PID
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShoot();
        // limelight/photon vision end
        Log.info("CmdShootDist", "Cancelling shooting");
    }
    
}
