package frc.team3128;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team3128.common.hardware.input.NAR_Joystick;
import frc.team3128.common.hardware.limelight.LEDMode;
import frc.team3128.common.hardware.limelight.Limelight;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.common.utility.Log;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private NAR_Joystick m_leftStick;
    private NAR_Joystick m_rightStick;

    private CommandScheduler m_commandScheduler = CommandScheduler.getInstance();
  
    private boolean DEBUG = true; 

    private Trigger isShooting;

    public RobotContainer() {
        // ConstantsInt.initTempConstants();

        //TODO: Enable all PIDSubsystems so that useOutput runs here

        m_leftStick = new NAR_Joystick(0);
        m_rightStick = new NAR_Joystick(1);

        // TODO: default driving command here

        initDashboard();
        configureButtonBindings();
        
        if(RobotBase.isSimulation())
            DriverStation.silenceJoystickConnectionWarning(true);
    }   

    private void configureButtonBindings() {

    }

    public void init() {

    }

    private void initDashboard() {
        if (DEBUG) {
            SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
        }

        NarwhalDashboard.startServer();   
        
        Log.info("NarwhalRobot", "Setting up limelight chooser...");
      
        for (Limelight ll : new Limelight[] {}) {
            NarwhalDashboard.addLimelight(ll);
            ll.setLEDMode(LEDMode.OFF);
        }
    }

    public void updateDashboard() {
        NarwhalDashboard.put("time", Timer.getMatchTime());
        NarwhalDashboard.put("voltage", RobotController.getBatteryVoltage());
    }
}
