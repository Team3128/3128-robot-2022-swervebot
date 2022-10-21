package frc.team3128.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.common.hardware.PicoColorSensor;
import frc.team3128.common.hardware.PicoColorSensor.RawColor;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonSRX;

import static frc.team3128.Constants.HopperConstants.*;

import java.io.FileDescriptor;

import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * Class for the Hopper Subsystem 
 */

public class Hopper extends SubsystemBase {

    private static Hopper instance;

    private PicoColorSensor m_colorSensor;
    private DigitalInput m_topDistanceSensor, m_bottomDistanceSensor;

    private NAR_TalonSRX m_hopper1, m_hopperShoot, m_hopperOuttake, m_hopperSerializer;



    public Hopper() {
        configMotors();
        configSensors();
    } 

    public static synchronized Hopper getInstance() {
        if (instance == null) 
            instance = new Hopper();
        return instance;
    }

    private void configMotors() {
        m_hopper1 = new NAR_TalonSRX(HOPPER_MOTOR_1_ID);
        m_hopperOuttake = new NAR_TalonSRX(HOPPER_MOTOR_OUTTAKE_ID);
        m_hopperSerializer = new NAR_TalonSRX(HOPPER_MOTOR_SERIALIZER_ID);
        m_hopperShoot = new NAR_TalonSRX(HOPPER_MOTOR_SHOOT_ID);

        m_hopper1.setNeutralMode(NeutralMode.Coast);
        m_hopperShoot.setNeutralMode(NeutralMode.Coast);
        m_hopperOuttake.setNeutralMode(NeutralMode.Coast);
        m_hopperSerializer.setNeutralMode(NeutralMode.Coast);
        m_hopperShoot.setNeutralMode(NeutralMode.Coast);
    }

    private void configSensors() {
        m_colorSensor = new PicoColorSensor();
        m_bottomDistanceSensor = new DigitalInput(0);
        m_topDistanceSensor = new DigitalInput(1);
    }
    
    /**
     * Hopper Methods
     */
    
     //Run Hopper
    public void runHopper() {
        m_hopper1.set(HOPPER_MOTOR_POWER);
        m_hopperSerializer.set(HOPPER_SERIALIZER_POWER);
        m_hopperOuttake.set(HOPPER_MOTOR_POWER);
    }

    //Outtake Hopper
    public void outtakeHopper() {
        m_hopper1.set(HOPPER_MOTOR_POWER);
        m_hopperSerializer.set(HOPPER_SERIALIZER_POWER);
        m_hopperOuttake.set(HOPPER_OUTTAKE_POWER);
    }

    public void outtakeHopper(double outtakePower) {
        m_hopper1.set(HOPPER_MOTOR_POWER);
        m_hopperSerializer.set(HOPPER_MOTOR_POWER);
        m_hopperOuttake.set(outtakePower);
    }

    //Reverse Hopper
    public void reverseHopper() {
        m_hopper1.set(-HOPPER_MOTOR_POWER);
        m_hopperSerializer.set(-HOPPER_SERIALIZER_POWER);
        m_hopperOuttake.set(-HOPPER_MOTOR_POWER);
    }

    //Stop hopper
    public void stopHopper() {
        m_hopper1.set(0);
        m_hopperSerializer.set(0);
        m_hopperOuttake.set(0);
    }

    public void runHopperShoot() {
        m_hopperShoot.set(HOPPER_SHOOT_POWER);
    }

    public void reverseHopperShoot() {
        m_hopperShoot.set(-HOPPER_SHOOT_POWER);
    }

    public void stopHopperShoot() {
        m_hopperShoot.set(0);
    }

     /** 
     * Color Sensor Methods
     */

    public boolean getBallBottomLocation() {
        return m_bottomDistanceSensor.get();
    }

    public boolean getBallTopLocation() {
        return m_topDistanceSensor.get();
    }

    /**
     * Returns true if bottom ball is wrong color, false if ball is right color or missing
     */
    public boolean getWrongBallBottom() {
        RawColor color = m_colorSensor.getRawColor0();
        if (color.red > color.blue*COLOR_SENSOR_TOLERANCE) {
            //return if red
            return DriverStation.getAlliance() != DriverStation.Alliance.Red;
        } else if (color.blue > color.red*COLOR_SENSOR_TOLERANCE) {
            //return if blue
            return DriverStation.getAlliance() != DriverStation.Alliance.Blue;
        } 
        return false;
    }

    /**
     * Returns true if top ball is wrong color, false if ball is right color or missing
     */
    public boolean getWrongBallTop() {
        RawColor color = m_colorSensor.getRawColor1();
        if (color.red > color.blue*1.5) {
            //return if red
            return DriverStation.getAlliance() != DriverStation.Alliance.Red;
        } else if (color.blue > color.red*1.5) {
            //return if blue
            return DriverStation.getAlliance() != DriverStation.Alliance.Blue;
        }
        return false;
    }

}