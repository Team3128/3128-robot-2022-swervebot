package frc.team3128.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.team3128.subsystems.Swerve;

import static frc.team3128.Constants.SwerveConstants.*;

public class CmdInPlaceTurn extends PIDCommand {

    private double degrees;

    public CmdInPlaceTurn(double degrees) {
        super(
            new PIDController(turnKP,turnKF,turnKD),
            () -> MathUtil.inputModulus(Swerve.getInstance().getHeading(),-180,180),
            0,
            output -> Swerve.getInstance().drive(new Translation2d(), output + Math.copySign(turnKF,output), false),
            Swerve.getInstance()
        );

        m_controller.enableContinuousInput(-180, 180);
        m_controller.setTolerance(turnTolerance);

        this.degrees = degrees;
    }

    @Override
    public void initialize() {
        super.initialize();
        m_setpoint = ()-> MathUtil.inputModulus(Swerve.getInstance().getHeading() + degrees,-180,180);
    }

    @Override
    public boolean isFinished() {
        return m_controller.atSetpoint();
    }
}