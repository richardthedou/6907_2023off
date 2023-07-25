package frc.lib6907.devices.gyro;

public class BadGyro extends Gyro {

    @Override
    public void calibrate() throws Exception {
        return;
    }

    @Override
    protected double getRawAngle(AngleAxis axis) {
        return 0.0;
    }

    @Override
    public boolean isAlive() {
        return false;
    }

    @Override
    protected double getRawDPS(AngleAxis axis) {
        return 0;
    }
    
}
