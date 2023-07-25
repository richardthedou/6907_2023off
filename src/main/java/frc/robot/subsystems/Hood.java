package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib6907.loops.ILooper;
import frc.lib6907.loops.Loop;
import frc.lib6907.subsystem.Subsystem;
import frc.robot.Constants;
import frc.robot.util.Util;

public class Hood extends Subsystem{

    private static final double MAGIC_VEL = 6000.0;
    private static final double HOOD_MAX_POS = 30000.0;
    private static final double ALLOWABLE_ERROR = 300.0;
    private static final double
        HOOD_KP = 1.0,
        HOOD_KI = 0.0,
        HOOD_KD = 15.0,
        HOOD_KF = 0.0,
        HOOD_ARBFF = 0.03;

    public enum HoodState {
        HOME,
        SOFT_CALIB,
        AIMING
    }
    
    public static class PeriodicIO {
        // INPUTS
        public double hood_current, hood_err;
        public double hood_outperc;
        public double hood_postick;
        public double hood_veltick;

        // OUTPUTS
        public double hood_targetpos;
    }

    private TalonFX mHoodFX;
    private PeriodicIO mPeriodicIO;
    private HoodState mHoodState;
    private double lastTimestamp;
    private boolean calibrated = false;

    private static Hood mInstance;

    public synchronized static Hood getInstance() {
        if (mInstance == null) {
            mInstance = new Hood();
        }

        return mInstance;
    }
    
    private Hood(){
        mPeriodicIO = new PeriodicIO();
        mHoodFX = new TalonFX(Constants.Hood_ID);
        mHoodState = HoodState.HOME;

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.openloopRamp = 0.05;
        config.closedloopRamp = 0.05;
        config.forwardSoftLimitEnable = true;
        config.forwardSoftLimitThreshold = HOOD_MAX_POS;
        config.reverseSoftLimitEnable = true;
        config.reverseSoftLimitThreshold = 500.0;
        config.slot0.kP = HOOD_KP;
        config.slot0.kI = HOOD_KI;
        config.slot0.kD = HOOD_KD;
        config.slot0.kF = HOOD_KF;
        config.peakOutputForward = 0.3;
        config.peakOutputReverse = -0.3;
        config.motionCruiseVelocity = MAGIC_VEL;
        config.motionAcceleration = MAGIC_VEL * 12.0;
        config.statorCurrLimit.enable = true;
        config.statorCurrLimit.currentLimit = 25.0;
        config.statorCurrLimit.triggerThresholdCurrent = 30.0;
        config.statorCurrLimit.triggerThresholdTime = 0.5;
        config.supplyCurrLimit.enable = true;
        config.supplyCurrLimit.currentLimit = 25.0;
        config.supplyCurrLimit.triggerThresholdCurrent = 30.0;
        config.supplyCurrLimit.triggerThresholdTime = 0.5;

        mHoodFX.configAllSettings(config);
        Util.configStatusFrame(mHoodFX);
        mHoodFX.selectProfileSlot(0, 0);
        mHoodFX.configAllowableClosedloopError(0, 100.0, 10);
        mHoodFX.setNeutralMode(NeutralMode.Brake);
        mHoodFX.setInverted(true);

        stop();
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.hood_current = mHoodFX.getStatorCurrent();
        mPeriodicIO.hood_outperc = mHoodFX.getMotorOutputPercent();
        mPeriodicIO.hood_veltick = mHoodFX.getSelectedSensorVelocity();
        mPeriodicIO.hood_postick = mHoodFX.getSelectedSensorPosition();
        mPeriodicIO.hood_err = mPeriodicIO.hood_targetpos - mPeriodicIO.hood_postick;

    }

    @Override
    public void writePeriodicOutputs() {
        if (mHoodState == HoodState.HOME) {
            mHoodFX.set(ControlMode.Position, 1000.0, DemandType.ArbitraryFeedForward, HOOD_ARBFF);
        } else if (mHoodState == HoodState.AIMING) {
            mHoodFX.set(ControlMode.Position, mPeriodicIO.hood_targetpos, DemandType.ArbitraryFeedForward, HOOD_ARBFF);
        } else if (mHoodState == HoodState.SOFT_CALIB) {
            mHoodFX.set(ControlMode.PercentOutput, -0.1);
        }
    }

    private void updateSoftCalib() {
        final double THRESHOLD = 150.0;
        if (Timer.getFPGATimestamp() - lastTimestamp < 0.5)
            return;
        
        if (Math.abs(mPeriodicIO.hood_veltick) < THRESHOLD) {
            mHoodFX.setSelectedSensorPosition(0, 0, 10);
            mHoodFX.configReverseSoftLimitEnable(true);
            mHoodState = HoodState.HOME;
            calibrated = true;
        }
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
                synchronized(Hood.this) {
                    setSoftCalib();
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Hood.this) {
                    switch(mHoodState) {
                        case SOFT_CALIB:
                            updateSoftCalib();
                            break;
                        case HOME:
                            break;
                        case AIMING:
                            break;
                        default:
                            mHoodState = HoodState.HOME;
                            break;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
            
        });
    }

    @Override
    public synchronized void stop() {
        if (mHoodState != HoodState.SOFT_CALIB)
            mHoodState = HoodState.HOME;
    }

    public synchronized void setSoftCalib() {
        if (mHoodState != HoodState.SOFT_CALIB) {
            mHoodState = HoodState.SOFT_CALIB;
            mHoodFX.configReverseSoftLimitEnable(false);
            lastTimestamp = Timer.getFPGATimestamp();
            calibrated = false;
        }
    }

    public synchronized void setHood(double target) {
        if (calibrated && mHoodState != HoodState.AIMING) {
            mHoodState = HoodState.AIMING;
        }
        mPeriodicIO.hood_targetpos = target;
    }

    public double getHoodPos() {
        return mPeriodicIO.hood_postick;
    }

    public double getHoodTarget() {
        return mPeriodicIO.hood_targetpos;
    }

    public boolean getCalibrated() {
        return calibrated;
    }

    public boolean isMagicInPosition() {
        return Math.abs(mPeriodicIO.hood_postick - mPeriodicIO.hood_targetpos) < ALLOWABLE_ERROR;
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        if (Constants.kSuperStructureDebug) {
            SmartDashboard.putString("HOOD STATE", mHoodState.toString());
            SmartDashboard.putNumber("HOOD POSTICK", mPeriodicIO.hood_postick);
            SmartDashboard.putNumber("HOOD TARGETPOS", mPeriodicIO.hood_targetpos);
            SmartDashboard.putNumber("HOOD CURR", mPeriodicIO.hood_current);
            SmartDashboard.putNumber("HOOD OUTPERC", mPeriodicIO.hood_outperc);
            SmartDashboard.putBoolean("HOOD CALIB", calibrated);
            SmartDashboard.putNumber("HOOD VELTICK", mPeriodicIO.hood_veltick);
        }
    }
    
}
