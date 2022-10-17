package frc.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.subsystems.Intake;
import frc.team3128.Constants.IntakeConstants;

public class CmdExtendIntake extends WaitCommand{
    private Intake m_intake; 

    public CmdExtendIntake(Intake intake){
        super(0.125); 
        m_intake = intake; 
        
        addRequirements(m_intake);
    }

    @Override
    public void initialize(){
        m_intake.reverseIntake(IntakeConstants.ARM_OUTTAKE_MOTOR_POWER);
        //intake
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
