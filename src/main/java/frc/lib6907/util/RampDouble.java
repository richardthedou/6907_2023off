package frc.lib6907.util;

public class RampDouble {
    
    private double mRamp;
    private double lastValue;

    public RampDouble(double ramp) {
        mRamp = ramp;
        lastValue = 0.0;
    }

    public double update(double value) {
        double newValue = limit(value, lastValue - mRamp, lastValue + mRamp);

        lastValue = newValue;
        return newValue;
    }

    private double limit(double v, double min, double max) {
        return Math.min(max, Math.max(v, min));
    }

}
