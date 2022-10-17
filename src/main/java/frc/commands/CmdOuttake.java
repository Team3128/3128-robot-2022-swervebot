package frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.subsystems.Intake;
import frc.team3128.Constants.IntakeConstants;

public class CmdOuttake extends CommandBase{
    private Intake m_intake; 
    private double intakePower; 

    public CmdOuttake(Intake intake){
        m_intake = intake; 
        intakePower = IntakeConstants.ARM_OUTTAKE_MOTOR_POWER; 
        
        addRequirements(m_intake);
    }

    public CmdOuttake(Intake intake, double intakePower){
        m_intake = intake;
        this.intakePower = intakePower; 
        
        addRequirements(m_intake);
    }

    @Override
    public void initialize(){
        m_intake.reverseIntake(intakePower);
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
