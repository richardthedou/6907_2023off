package frc.lib6907.io;

import edu.wpi.first.wpilibj.XboxController;
import frc.lib6907.geometry.GRotation2d;
import frc.lib6907.geometry.GTranslation2d;
import frc.lib6907.util.RampDouble;

public abstract class SwerveController {

    private static final double DEADBAND = 0.12;
    protected static final double NEAR_POLE_THRESHOLD = Math.toRadians(20.0);
    protected XboxController mXbox;
    private RampDouble mDvRamp;
    protected double currHeading = 0.0;
    protected DriveViewComp mViewComp = DriveViewComp.MIDDLE;
    private ButtonCheck buttonA, buttonB;
    private GRotation2d lastPosRot;

    public enum DriveViewComp {
        LEFT(-20.0),
        MIDDLE(0.0),
        RIGHT(20.0);

        public double viewCompAngle;
        private DriveViewComp(double comp) {
            viewCompAngle = comp;
        }
    }

    public SwerveController(int port) {
        mXbox = new XboxController(port);
        currHeading = 0.0;
        mDvRamp = new RampDouble(0.009);
        buttonA = new ButtonCheck();
        buttonB = new ButtonCheck();
        lastPosRot = new GRotation2d();
    }

    public void setDriveViewComp(DriveViewComp dvComp) {
        if (dvComp != mViewComp)
            mViewComp = dvComp;
    }

    public GTranslation2d getDriveVector() {
        GTranslation2d dv = new GTranslation2d(-mXbox.getLeftY(), -mXbox.getLeftX());
        dv = dv.rotateBy(GRotation2d.fromDegrees(mViewComp.viewCompAngle));
        double n = mDvRamp.update(dv.getNorm());
        if (dv.getNorm() < DEADBAND) {
            dv = lastPosRot.toTranslation().scale(n);
        } else {
            lastPosRot = dv.direction();
            dv = dv.scale(n / dv.getNorm());
        }
        // POLE ANGLE ADJUST
        if(Math.abs(dv.direction().distance(dv.direction().nearestPole())) < NEAR_POLE_THRESHOLD)
            dv = dv.direction().nearestPole().toTranslation().scale(dv.getNorm());
        return dv;
    }

    public boolean getDriveLowPower() {
        return mXbox.getRightBumper();
    }

    // INFINITE FOR NOT NEEDED
    public double getDriveHeadingDegree() {
        buttonA.update(mXbox.getAButton());
        buttonB.update(mXbox.getBButton());
        if (buttonB.shortReleased())
            currHeading = 22.50 + 90.0;
        else if (buttonB.longPressed())
            currHeading = 22.50;
        else if (buttonA.longPressed())
            currHeading = 14.58 + 90.0;
        else if (buttonA.shortReleased())
            currHeading = 22.50 + 180;
        else 
            currHeading = getInputHeadingDegree();
        return currHeading;
    }

    // INFINITE FOR NOT NEEDED
    public abstract double getInputHeadingDegree();

    public abstract double getRawChangeRate();

    public XboxController getXbox() {
        return mXbox;
    }

}