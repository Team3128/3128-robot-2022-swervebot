package frc.team3128.common.hardware.limelight;

import edu.wpi.first.math.util.Units;

public enum Camera {
    SHOOTER("limelight-cog",true,10000,40, Units.inchesToMeters(104)), //Bad Numbers
    PI_CAMERA("teja",false,10000, 40, Units.inchesToMeters(9.5/2)); //Bad Numbers

    public String hostname;

    public boolean LED;

    public double cameraHeight;

    public double cameraAngle;

    public double targetHeight;

    private Camera(String hostname, boolean LED, double cameraHeight, double cameraAngle, double targetHeight) {
        this.hostname = hostname;
        this.LED = LED;
        this.cameraHeight = cameraHeight;
        this.cameraAngle = cameraAngle;
        this.targetHeight = targetHeight;
    }

}