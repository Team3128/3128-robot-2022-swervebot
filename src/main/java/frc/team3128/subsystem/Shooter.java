package frc.team3128.subsystem;

import com.ctre.phoenix.motorcontrol.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.team3128.Constants.ConversionConstants;
import frc.team3128.Constants.ShooterConstants;
import edu.wpi.first.math.controller.PIDController;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonFX;

public class Shooter extends PIDSubsystem {

    private NAR_TalonFX m_leftShooter, m_rightShooter;

    private double thresholdPrecent = ShooterConstants.RPM_THRESHOLD_PRECENT;

    // private SimpleMotorFeedforward LowFF = new
    // SimpleMotorFeedforward(ShooterConstants.LOW_kS, ShooterConstants.LOW_kV,
    // ShooterConstants.LOW_kA);
    // private SimpleMotorFeedforward HighFF = new
    // SimpleMotorFeedforward(ShooterConstants.HIGH_kS, ShooterConstants.HIGH_kV,
    // ShooterConstants.HIGH_kA);

    private double time = 0, prevTime = 0;

    private double plateauConunt = 0;
    private double plateauThreshold = ShooterConstants.PLATEAU_THRESHOLD;

    private static Shooter instance;

    public Shooter() {
        super(new PIDController(
                ShooterConstants.HIGH_kP,
                ShooterConstants.HIGH_kI,
                ShooterConstants.HIGH_kD),
                ShooterConstants.PLATEUA_COUNT);

        motorConfig();
    }

    private void motorConfig() {
        m_leftShooter = new NAR_TalonFX(ShooterConstants.LEFT_SHOOTER_ID);
        m_rightShooter = new NAR_TalonFX(ShooterConstants.RIGHT_SHOOTER_ID);

        m_leftShooter.setInverted(false);
        m_rightShooter.setInverted(true);

        m_rightShooter.follow(m_leftShooter);

        m_leftShooter.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 17);
        m_leftShooter.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 47);

        m_rightShooter.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
        m_rightShooter.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);

        m_leftShooter.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(
                        true,
                        15,
                        30,
                        0.1));
    }

    public static synchronized Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    public void beginShoot(double RPM) {
        thresholdPrecent = ShooterConstants.RPM_THRESHOLD_PRECENT;
        super.setSetpoint(RPM);
        getController().setTolerance(thresholdPrecent * RPM);
    }

    public void stopShoot() {
        setSetpoint(0);
        resetPlateauCount();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        double ff = setpoint * ShooterConstants.kF;
        double outputVoltage = ff + output;
        time = RobotController.getFPGATime() / 1e6;
        if (thresholdPrecent < ShooterConstants.RPM_THRESHOLD_PRECENT_MAX) {thresholdPrecent += (time - prevTime) * (ShooterConstants.RPM_THRESHOLD_PRECENT_MAX - ShooterConstants.RPM_THRESHOLD_PRECENT) / ShooterConstants.TIME_TO_MAX_THRESHOLD;
            getController().setTolerance(thresholdPrecent * setpoint);
        }
    }

    @Override
    protected double getMeasurement() {
        return m_leftShooter.getSelectedSensorPosition() * ConversionConstants.FALCON_NUpS_TO_RPM;
    }

    private void resetPlateauCount() {
        plateauConunt = 0;
    }

    private void checkPlateau(double setpoint, double plateauTolerance) {
        if (Math.abs(setpoint - getMeasurement()) <= (setpoint * plateauTolerance) && (setpoint != 0)) {
            plateauConunt++;
        } else {
            plateauConunt = 0;
        }
    }

    public boolean isReady() {
        return (plateauConunt >= plateauThreshold);
    }
}
