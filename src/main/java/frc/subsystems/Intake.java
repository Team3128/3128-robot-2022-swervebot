package frc.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonFX;
import net.thefletcher.revrobotics.enums.MotorType;
import frc.team3128.Constants.IntakeConstants;

/*
* Intake subsystem for swervebot 2022
* @author Kailani Minna-Choe
*/

public class Intake extends PIDSubsystem{ 

    private NAR_CANSparkMax m_armMotor; 
    private NAR_TalonFX m_brushMotor; 
    private double tolerance = IntakeConstants.TOLERANCE_MIN; 
    //private double time;
    //private double prevTime; 
//one ff
//log vbalues talk mason shulffebaord

    private static Intake instance; 

    public static synchronized Intake getInstance() {
        if (instance == null) {
            instance = new Intake(); 
        }
        return instance; 
    }

    public Intake(){
        super(new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD), IntakeConstants.MIN_ANGLE);
        configMotors(); 
    }

    private void configMotors(){
        m_armMotor = new NAR_CANSparkMax(IntakeConstants.ARM_MOTOR_ID, MotorType.kBrushless); 
        m_brushMotor = new NAR_TalonFX(IntakeConstants.BRUSH_MOTOR_ID); 
    }

    //shuffleboard
    public void initShuffleboard(){
        //General tab
        //NAR_Shuffleboard.addData("General", "Intake Speed", m_intake::get).withPosition
        //Intake Tab
        //NAR_Shuffleboard.addData(); 
    }

    public void resetEncoders(){
        m_armMotor.setEncoderPosition(0);
    }

    //Runs brush motor
    public void runIntake(){
        m_brushMotor.set(IntakeConstants.BRUSH_MOTOR_POWER); 
    }

    //stops brush motor
    public void stopIntake(){
        m_brushMotor.set(0); 
    }

    //Runs brush motor backward
    public void reverseIntake(double power){
        m_brushMotor.set(-IntakeConstants.BRUSH_MOTOR_POWER); //double check if this is okay
    }

    //PID for arm motor
    public void startPID(double angle) {
        tolerance = IntakeConstants.TOLERANCE_MIN; 
        super.setSetpoint(angle);
        getController().setTolerance(tolerance);
    }

    //Runs arm motor to PID
    @Override
    protected void useOutput(double output, double setpoint) {
        setpoint = m_armMotor.getSetpoint(); 
        double ff = IntakeConstants.kF * Math.cos(Units.degreesToRadians(setpoint)); //does need cos
        double voltageOutput = output + ff; 
        
        m_armMotor.set(MathUtil.clamp(voltageOutput/12.0, -1, 1)); 
    }

    //Gets input of arm motor position
    @Override
    protected double getMeasurement() {
        return m_armMotor.getSelectedSensorPosition() + IntakeConstants.MIN_ANGLE; 
    }

    //
    public void extendIntake(){
        startPID(IntakeConstants.MAX_ANGLE); 
    }

    public void retractIntake(){
        startPID(IntakeConstants.MIN_ANGLE); 
    }
}
