package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;
import frc.lib6907.geometry.GTranslation2d;

public class SwerveControllerTest {
    private XboxController mXbox;
    private double DEADBAND = 0.08;
    private double ROTATE_DEADBAND = 0.7;
    private double NEAR_POLE_THRESHOLD = 7;

    public SwerveControllerTest(int port) {
        mXbox = new XboxController(port);
    }

    public GTranslation2d getDriveVector() {
        GTranslation2d dv = new GTranslation2d(-mXbox.getLeftY(), -mXbox.getLeftX());
        if (dv.getNorm() < DEADBAND)
            dv = new GTranslation2d(0, 0);
        else{
            dv = dv.minus(dv.scaledTo(DEADBAND)).scale(1/(1-DEADBAND));
        }

        //Near pole adjust
        if(Math.abs(dv.direction().distanceDeg(dv.direction().nearestPole())) < NEAR_POLE_THRESHOLD)
            dv = new GTranslation2d(dv.getNorm(), dv.direction().nearestPole());

        return dv;
    }
    public double getRawChangeRate(){
        double rotate = Util.eliminateDeadband(mXbox.getLeftTriggerAxis() - mXbox.getRightTriggerAxis());
        return rotate;
    }
    public boolean getPigeonReset(){
        boolean isReset = mXbox.getStartButtonPressed();
        return isReset;
    }

    public double getDriveTargetAngle() {
        double targetAngle = 0;
        GTranslation2d headingVector = new GTranslation2d(-mXbox.getRightY(), -mXbox.getRightX());
        if (headingVector.getNorm() < ROTATE_DEADBAND) {
            targetAngle = Double.POSITIVE_INFINITY;
        } else {
            targetAngle = headingVector.direction().getDegrees();
        }
        return targetAngle;
    }

    public boolean getRightStick() {
        return mXbox.getRightStickButton();
    }

    public boolean getSlowMode(){
        return mXbox.getRightBumper();
    }

    public boolean getLeftBumperPressed() {
        return mXbox.getLeftBumperPressed();
    }

    public boolean getVisionAimPressed() {
        return mXbox.getYButtonPressed();
    }

    //FOR TEST ONLY
    public XboxController getDriver() {
        return mXbox;
    }

    public boolean getAButtonPressed(){
        return mXbox.getAButtonPressed();
    }

    public double getPOV() {
        return mXbox.getPOV();
    }
}
