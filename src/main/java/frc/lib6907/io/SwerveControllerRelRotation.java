package frc.lib6907.io;

import frc.robot.util.Util;

public class SwerveControllerRelRotation extends SwerveController {
    
    public SwerveControllerRelRotation(int port) {
        super(port);
    }

    // raw rate is doing everything
    // not needed here
    @Override
    public double getInputHeadingDegree() {
        return Double.POSITIVE_INFINITY;        // currHeading
    }

    @Override
    public double getRawChangeRate() {
        double in = Util.eliminateDeadband(-mXbox.getRightX());
        // INPUT NONLINEARITY
        return Math.pow(Math.abs(in), 1.75)*Math.signum(in);
    }
    
}