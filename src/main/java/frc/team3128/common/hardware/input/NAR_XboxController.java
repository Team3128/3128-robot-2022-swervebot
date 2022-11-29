package frc.team3128.common.hardware.input;

import java.util.HashMap;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class NAR_XboxController extends XboxController {

    private String buttonNames[] = {
        "A",
        "B",
        "X",
        "Y",
        "LeftBumper",
        "RightBumper",
        "Back",
        "Start",
        "LeftStick",
        "RightStick"
    };
    
    private HashMap<String, Trigger> buttons;

    public NAR_XboxController(int port) {
        super(port);
        buttons = new HashMap<String, Trigger>();
        for (int i = 0; i < 10; i++) {
            int n = i + 1;
            buttons.put(buttonNames[i], new Trigger (() -> this.getRawButton(n)));
        }   
    }

    public Trigger getButton(String buttonName) {
        return buttons.get(buttonName);
    }

    public Trigger getLeftTrigger() {
        return new Trigger (() -> getLeftTriggerAxis() >= 0.5);
    }

    public Trigger getRightTrigger() {
        return new Trigger (() -> getRightTriggerAxis() >= 0.5);
    }

    @Override
    public double getRightX() {
        return Math.abs(super.getRightX()) > 0.1 ? -super.getRightX():0;
    }

    @Override
    public double getRightY() {
        return Math.abs(super.getRightY()) > 0.1 ? -super.getRightY():0;
    }

    @Override
    public double getLeftX() {
        return Math.abs(super.getLeftX()) > 0.1 ? super.getLeftX():0;
    }

    @Override
    public double getLeftY() {
        return Math.abs(super.getLeftY()) > 0.1 ? -super.getLeftY():0;
    }
    
}