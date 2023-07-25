package frc.lib6907.devices.gyro;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

public class Pigeon extends Gyro {

    private PigeonIMU pigeon;

    public Pigeon(int port) {
        pigeon = new PigeonIMU(port);
        pigeon.configFactoryDefault();
    }

    public Pigeon(TalonSRX talon) {
        pigeon = new PigeonIMU(talon);
        pigeon.configFactoryDefault();
    }

    @Override
    public void calibrate() throws Exception {
        pigeon.enterCalibrationMode(CalibrationMode.Accelerometer, 10);
    }

    @Override
    public double getRawAngle(AngleAxis axis) {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        switch(axis) {
            case Roll:
                return ypr[1];
            case Pitch:
                return ypr[2];
            case Yaw:
            default:
                return pigeon.getFusedHeading();     // shift with mag
        }
    }

    public double getRawDPS(AngleAxis axis) {
        double[] xyz_dps = new double[3];
        pigeon.getRawGyro(xyz_dps);
        switch (axis) {
            case Roll:
                return xyz_dps[0];
            case Pitch:
                return xyz_dps[1];
            case Yaw:
                return xyz_dps[2];
            default:
                return xyz_dps[2];
        }
    }


    @Override
    public boolean isAlive() {
        return pigeon.getState() == PigeonState.Ready;
    }
    
}
