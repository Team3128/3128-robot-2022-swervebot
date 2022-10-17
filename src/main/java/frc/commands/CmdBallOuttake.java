package frc.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.subsystems.Intake;

public class CmdBallOuttake extends WaitCommand{
    private Intake m_intake; 

    public CmdBallOuttake(Intake intake){
        super(0.125); //seconds
        m_intake = intake; 
        
        addRequirements(m_intake);
    }

    @Override
    public void initialize(){
        super.initialize();
        m_intake.extendIntake();
    }

    @Override
    public void end(boolean interruped){
        m_intake.stopIntake();
        m_intake.retractIntake();
    }

    public boolean isFinished(){
        return false; 
    }
}
