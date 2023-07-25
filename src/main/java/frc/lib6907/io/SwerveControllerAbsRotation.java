package frc.lib6907.io;

import frc.lib6907.geometry.GRotation2d;
import frc.lib6907.geometry.GTranslation2d;
import frc.robot.util.Util;

public class SwerveControllerAbsRotation extends SwerveController {

    private static final double NORM_THRESHOLD = 0.8;

    public SwerveControllerAbsRotation(int port) {
        super(port);
    }

    @Override
    public double getInputHeadingDegree() {
        GTranslation2d v = new GTranslation2d(-mXbox.getRightY(), -mXbox.getRightX());
        if (v.getNorm() > NORM_THRESHOLD) {
            // POLE ANGLE ADJUST
            if(Math.abs(v.direction().distance(v.direction().nearestPole())) < NEAR_POLE_THRESHOLD){
                v = v.direction().nearestPole().toTranslation();
            }
                
            v = v.rotateBy(GRotation2d.fromDegrees(mViewComp.viewCompAngle));
            return v.direction().getDegrees();
        } else 
            return Double.POSITIVE_INFINITY;
    }

    @Override
    public double getRawChangeRate() {
        double in = Util.eliminateDeadband(mXbox.getLeftTriggerAxis() - mXbox.getRightTriggerAxis());
        in *= 0.6;
        // INPUT NONLINEARITY
        return Math.pow(Math.abs(in), 1.75)*Math.signum(in);
    }
    
}
