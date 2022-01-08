package frc.team3128;

import frc.team3128.subsystems.*;
import frc.team3128.commands.*;
import frc.team3128.common.hardware.input.NAR_Joystick;
import frc.team3128.common.hardware.limelight.LEDMode;
import frc.team3128.common.hardware.limelight.Limelight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private NAR_Drivetrain m_drive;

    private Limelight limelight = new Limelight("limelight-sog", -26.0, 0, 0, 30);

    private NAR_Joystick m_leftStick;
    private NAR_Joystick m_rightStick;

    private CommandScheduler m_commandScheduler = CommandScheduler.getInstance();
    private Command auto;

    public RobotContainer() {

        m_drive = NAR_Drivetrain.getInstance();

        //Enable all PIDSubsystems so that useOutput runs

        m_leftStick = new NAR_Joystick(0);
        m_rightStick = new NAR_Joystick(1);

        limelight.setLEDMode(LEDMode.ON);

        m_commandScheduler.setDefaultCommand(m_drive, new ArcadeDrive(m_drive, m_rightStick::getY, m_rightStick::getTwist, m_rightStick::getThrottle));

        configureButtonBindings();
        dashboardInit();
    }   

    private void configureButtonBindings() {
        
    }

    private void dashboardInit() {
        SmartDashboard.putData(CommandScheduler.getInstance());
        
        SmartDashboard.putData(m_drive);

    }

    public void stopDrivetrain() {
        m_drive.stop();
    }

    public Command getAutonomousCommand() {
        return auto;
    }
}
