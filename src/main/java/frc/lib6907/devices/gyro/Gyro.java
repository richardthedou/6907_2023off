package frc.lib6907.devices.gyro;

import frc.lib6907.util.CircularBuffer;

public abstract class Gyro {
    
    public enum AngleAxis {
        Roll,
        Pitch,
        Yaw
    }

    private double angleOffset = 0.0;
    private boolean reverseAngle = false;
    private AngleAxis mAngleAxis = AngleAxis.Yaw;
    private CircularBuffer filter = new CircularBuffer(5);
    private CircularBuffer dpsFilter = new CircularBuffer(5);
    private double lastAngle = 0.0;
    private double lastDPS = 0.0;

    public abstract void calibrate() throws Exception;

    protected abstract double getRawAngle(AngleAxis axis);

    protected abstract double getRawDPS(AngleAxis axis);

    public abstract boolean isAlive();

    public void setAngleAxis(AngleAxis axis) {
        if (axis != null)
            mAngleAxis = axis;
    }

    public void setReversed(boolean rev) {
        reverseAngle = rev;
    }

    public void reset() {
        reset(0.0);
    }

    public void reset(double initAngle) {
        angleOffset = getRawAngle() + initAngle * (reverseAngle ? 1.0 : -1.0);
        lastAngle = initAngle;
        filter.clear();
    }

    private double getRawAngle() {
        return getRawAngle(mAngleAxis);
    }



    public synchronized double getAngle() {
        double angle = (getRawAngle() - angleOffset) * (reverseAngle ? -1.0 : 1.0);
        if (!isAlive())
            return lastAngle;
        filter.addValue(placeInAppropriate0To360Scope(lastAngle, angle));
        lastAngle = filter.getAverage();
        return lastAngle;
    }

    public synchronized double getFilteredYawDPS() {
        double dps = getRawDPS(AngleAxis.Yaw) * (reverseAngle ? -1.0 : 1.0);
        if(!isAlive())
            return lastDPS;
        dpsFilter.addValue(dps);
        lastDPS = dpsFilter.getAverage();
        return lastDPS;
    }

    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle){
    	double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if(lowerOffset >= 0){
        	lowerBound = scopeReference - lowerOffset;
        	upperBound = scopeReference + (360 - lowerOffset);
        }else{
        	upperBound = scopeReference - lowerOffset; 
        	lowerBound = scopeReference - (360 + lowerOffset);
        }
        while(newAngle < lowerBound){
        	newAngle += 360; 
        }
        while(newAngle > upperBound){
        	newAngle -= 360; 
        }
        if(newAngle - scopeReference > 180){
        	newAngle -= 360;
        }else if(newAngle - scopeReference < -180){
        	newAngle += 360;
        }
        return newAngle;
    }

}
