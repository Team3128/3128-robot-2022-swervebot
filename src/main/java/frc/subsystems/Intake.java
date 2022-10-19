package frc.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.team3128.common.hardware.motorcontroller.NAR_CANSparkMax;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonFX;
import net.thefletcher.revrobotics.enums.MotorType;
import frc.team3128.Constants.IntakeConstants; 
import frc.team3128.common.utility.NAR_Shuffleboard; 

/*
* Intake subsystem for swervebot 2022
* @author Kailani Minna-Choe
* @since 2022 Rapid React offseason
*/

public class Intake extends PIDSubsystem{ 

    private NAR_CANSparkMax m_armMotor; 
    private NAR_TalonFX m_brushMotor; 
    private double tolerance = IntakeConstants.TOLERANCE_MIN; 
    private double m_ff; 

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
        //General - wait until with full robot
        //NAR_Shuffleboard.addData("General", "Intake Brush Speed", m_brushMotor.get()); 
        //withPosition(x, y); -> for logging 
        //Intake Tab
        NAR_Shuffleboard.addData("Intake", "Intake Brush Speed", m_brushMotor.get()); 
        //PID Arm 
        NAR_Shuffleboard.addData("Intake", "Intake Arm Speed", m_armMotor.get()); 
        NAR_Shuffleboard.addComplex("Intake PID Arm", "Intake Arm", this); 
        NAR_Shuffleboard.addData("Intake PID Arm", "Setpoint", m_armMotor.getSetpoint()); 
        NAR_Shuffleboard.addData("Intake PID Arm", "Angle", getMeasurement()); 
        NAR_Shuffleboard.addComplex("Intake PID Arm", "PID", this.m_controller); 
        NAR_Shuffleboard.debug("Intake PID Arm", "ff", IntakeConstants.kF, IntakeConstants.x, 0);
        NAR_Shuffleboard.debug("Intake PID Arm", "set angle", IntakeConstants.kF, IntakeConstants.x, 1);
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
        m_ff = IntakeConstants.kF * Math.cos(Units.degreesToRadians(setpoint)); //does need cos
        double voltageOutput = output + m_ff; 
        
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
