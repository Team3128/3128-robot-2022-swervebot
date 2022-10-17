package frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Intake;

public class CmdBallIntake extends CommandBase{
    private Intake m_intake; //also add hopper

    public CmdBallIntake(Intake intake){
        m_intake = intake; 
        
        addRequirements(m_intake);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void end(boolean interruped){
        m_intake.retractIntake();
        m_intake.stopIntake();
    }

    public boolean isFinished(){
        return false; 
    }
}
