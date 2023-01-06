package frc.team3128;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.team3128.Constants.VisionConstants.*;
import frc.team3128.autonomous.Trajectories;
import frc.team3128.commands.CmdAlign;
import frc.team3128.commands.CmdInPlaceTurn;
import frc.team3128.commands.CmdSwerveDrive;
import frc.team3128.commands.CmdTargetPursuit;
import frc.team3128.common.hardware.input.NAR_Joystick;
import frc.team3128.common.hardware.camera.Camera;
import frc.team3128.common.hardware.camera.NAR_Camera;
import frc.team3128.common.hardware.input.NAR_XboxController;
import frc.team3128.common.narwhaldashboard.NarwhalDashboard;
import frc.team3128.common.utility.Log;
import frc.team3128.subsystems.Swerve;
import frc.team3128.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private Swerve swerve;
    private Vision vision;
    private NAR_Camera cam;

    private NAR_Joystick leftStick;
    private NAR_Joystick rightStick;

    private NAR_XboxController controller;

    private CommandScheduler commandScheduler = CommandScheduler.getInstance();
  
    private boolean DEBUG = true; 

    private Trigger hasTarget;

    public RobotContainer() {
        vision = Vision.getInstance();
        // ConstantsInt.initTempConstants();
        swerve = Swerve.getInstance();

        //TODO: Enable all PIDSubsystems so that useOutput runs here

// 
        leftStick = new NAR_Joystick(0);
        rightStick = new NAR_Joystick(1);
        controller = new NAR_XboxController(2);

        commandScheduler.setDefaultCommand(swerve, new CmdSwerveDrive(rightStick::getX, rightStick::getY, rightStick::getZ, rightStick::getThrottle, true));
        //commandScheduler.setDefaultCommand(swerve, new CmdSwerveDrive(controller::getLeftX,controller::getLeftY, controller::getRightX, rightStick::getThrottle, true));
        initDashboard();
        configureButtonBindings();
        
        if(RobotBase.isSimulation())
            DriverStation.silenceJoystickConnectionWarning(true);
    }   

    private void configureButtonBindings() {
        rightStick.getButton(1).whenActive(new InstantCommand(()->swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))));
        rightStick.getButton(2).whenActive(new InstantCommand(swerve::toggle));
        rightStick.getButton(3).whenActive(new InstantCommand(()->swerve.resetOdometry(vision.robotPos(SHOOTER.hostname))));
        rightStick.getButton(4).whenActive(new CmdAlign(SHOOTER.hostname)).whenInactive(new InstantCommand(()-> swerve.stop()));
        rightStick.getButton(5).whenActive(new InstantCommand(()->swerve.zeroGyro(vision.robotPos(SHOOTER.hostname).getRotation().getDegrees())));
        rightStick.getButton(6).whenActive(new CmdTargetPursuit(SHOOTER.hostname,1.5)).whenInactive(new InstantCommand(()->swerve.stop(),swerve));
        rightStick.getButton(7).whenActive(new CmdInPlaceTurn(180,SHOOTER.hostname));
        rightStick.getButton(8).whenActive(new InstantCommand(()-> redBlueToggle()));
        rightStick.getButton(9).whenActive(new RunCommand(()-> Swerve.getInstance().drive(new Translation2d(0.1,0),0,false),Swerve.getInstance())).whenInactive(new InstantCommand(()->Swerve.getInstance().stop(),Swerve.getInstance()));
        hasTarget = new Trigger(()-> vision.hasValidTarget(SHOOTER.hostname))
        .whileActiveContinuous(new RunCommand(()-> controller.setRumble(RumbleType.kLeftRumble,0)))
        .whenInactive(new InstantCommand(()-> controller.setRumble(RumbleType.kLeftRumble, 0)));
    }

    public void init() {

    }

    public void redBlueToggle() {
        Robot.isRED = Robot.isRED == false;
    }

    private void initDashboard() {
        if (DEBUG) {
            SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
            //SmartDashboard.putData("Swerve", swerve);
        }

        NarwhalDashboard.startServer();   
        
        Log.info("NarwhalRobot", "Setting up limelight chooser...");
      
        for (NAR_Camera ll : vision.getCameras()) {
            NarwhalDashboard.addLimelight(ll);
            ll.setLED(false);
        }
    }

    public void updateDashboard() {
        SmartDashboard.putBoolean("ISRED", Robot.isRED);
        NarwhalDashboard.put("time", Timer.getMatchTime());
        NarwhalDashboard.put("voltage", RobotController.getBatteryVoltage());
        NarwhalDashboard.put("x", swerve.getPose().getX());
        NarwhalDashboard.put("y", swerve.getPose().getY());
    //     SmartDashboard.putNumber("LeftX",controller.getLeftX());
    //     SmartDashboard.putNumber("LeftY",controller.getLeftY());
    //     SmartDashboard.putNumber("RightX",controller.getRightX());
    //     SmartDashboard.putNumber("RightY",controller.getRightY());
    }
}
