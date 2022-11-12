package frc.team3128.common.hardware.limelight;

public enum Camera {
    SHOOTER("limelight-sog",true,0,0, 0, 0), //Bad Numbers
    PI_CAMERA("teja",false,10000, 40, 0,10); //Bad Numbers

    public String hostname;

    public boolean LED;

    public double cameraHeight; //inches

    public double cameraAngle;  //degrees

    public double targetHeight; //inches

    public double cameraOffset; //inches

    private Camera(String hostname, boolean LED, double cameraHeight, double cameraAngle, double targetHeight, double cameraOffset) {
        this.hostname = hostname;
        this.LED = LED;
        this.cameraHeight = cameraHeight;
        this.cameraAngle = cameraAngle;
        this.targetHeight = targetHeight;
        this.cameraOffset = cameraOffset;
    }

}
