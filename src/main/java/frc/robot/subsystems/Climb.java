package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib6907.loops.ILooper;
import frc.lib6907.loops.Loop;
import frc.lib6907.subsystem.Subsystem;
import frc.robot.Constants;
import frc.robot.util.Util;

public class Climb extends Subsystem {
    private static final double MAGIC_VEL = 8000.0;
    private static final double MAX_POS = 70600.0;
    private static final double ALLOWABLE_ERROR = 300.0;

    private static final double
        CLIMB_KP = 0.25,
        CLIMB_KI = 0.0,
        CLIMB_KD = 25.0,
        CLIMB_KF = 0.04,
        CLIMB_ARBFF = 0.0;    //gravity compensation
    
    public enum ClimbState {
        HOME,
        SOFT_CALIB,
        CLIMBING
    }

    public static class PeriodicIO {
        //inputs
        public double climb_curr, climb_err;
        public double climb_outperc;
        public double climb_postick;
        public double climb_veltick;

        //outputs
        public double climb_targetpos;
        public double turret_desiredpos;
    }

    public TalonFX mFX;
    private PeriodicIO mPIO;
    private ClimbState state;
    private double pretimeStamp;
    private boolean calibrated = false;

    private static Climb mIns;

    public synchronized static Climb getInstance() {
        if(mIns == null)    mIns = new Climb();
        return mIns;
    }

    private Climb() {
        mPIO = new PeriodicIO();
        mFX = new TalonFX(Constants.CLIMB_ID);
        state = ClimbState.HOME;

        //motor config
        TalonFXConfiguration config = new TalonFXConfiguration();   
        config.openloopRamp = 0.05;
        config.closedloopRamp = 0.05;
        config.forwardSoftLimitEnable = true;
        config.forwardSoftLimitThreshold = MAX_POS;
        config.reverseSoftLimitEnable = true;
        config.reverseSoftLimitThreshold = 100.0;
        config.slot0.kP = CLIMB_KP;
        config.slot0.kI = CLIMB_KI;
        config.slot0.kD = CLIMB_KD;
        config.slot0.kF = CLIMB_KF;
        config.peakOutputForward = 0.3;
        config.peakOutputReverse = -0.85;
        config.motionCruiseVelocity = MAGIC_VEL;
        config.motionAcceleration = MAGIC_VEL*4;
        config.statorCurrLimit.enable = false;
        config.statorCurrLimit.currentLimit = 25.0;
        config.statorCurrLimit.triggerThresholdCurrent = 30.0;
        config.statorCurrLimit.triggerThresholdTime = 0.5;
        config.supplyCurrLimit.enable = true;
        config.supplyCurrLimit.currentLimit = 40.0;
        config.supplyCurrLimit.triggerThresholdCurrent = 40.0;
        config.supplyCurrLimit.triggerThresholdTime = 0.5;


    

        mFX.configAllSettings(config);
        Util.configStatusFrame(mFX);
        mFX.selectProfileSlot(0, 0);
        mFX.configAllowableClosedloopError(0, 100.0, 10);
        mFX.setSelectedSensorPosition(0);
        mFX.setNeutralMode(NeutralMode.Brake);
        mFX.setInverted(true);

        stop();
    }

    @Override
    public void readPeriodicInputs() {
        mPIO.climb_curr = mFX.getStatorCurrent();
        mPIO.climb_outperc = mFX.getMotorOutputPercent();
        mPIO.climb_veltick = mFX.getSelectedSensorVelocity();
        mPIO.climb_postick = mFX.getSelectedSensorPosition();
        mPIO.climb_err = mPIO.climb_targetpos - mPIO.climb_postick;
    }

    @Override
    public void writePeriodicOutputs() {
        switch(state) {
            case HOME:
                mFX.set(ControlMode.MotionMagic, 1000.0, DemandType.ArbitraryFeedForward, CLIMB_ARBFF);
                break;
            case CLIMBING:
                mFX.set(ControlMode.MotionMagic, mPIO.climb_targetpos, DemandType.ArbitraryFeedForward, CLIMB_ARBFF);
                break;
            case SOFT_CALIB:
                mFX.set(ControlMode.PercentOutput, -0.1);
                break;
            default:
                break;
        }
    }

    public void updateSoftCalib() {
        final double THRESHOLD = 150.0;
        if(Timer.getFPGATimestamp() - pretimeStamp < 0.5) return ;
        if(Math.abs(mPIO.climb_veltick) < THRESHOLD) {
            mFX.setSelectedSensorPosition(0, 0, 10);
            mFX.configReverseSoftLimitEnable(true);
            state = ClimbState.HOME;
            calibrated = true;
        }
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Climb.this) {
                    setSoftCalib();
                    updateSoftCalib();
                }
            }

            @Override
            public void onLoop(double timeStamp) {
                synchronized(Climb.this) {
                    switch(state) {
                        case SOFT_CALIB:
                            updateSoftCalib();
                        case HOME:
                            break;
                        case CLIMBING:
                            updateClimbing();
                            break;
                        default:
                            state = ClimbState.HOME;
                            break;
                    }
                }
            }

            @Override
            public void onStop(double timeStamp) {
                synchronized(Climb.this) {
                    stop();
                }
            }
        });
    }

    @Override
    public synchronized void stop() {
        if(state != ClimbState.SOFT_CALIB)  state = ClimbState.HOME;
    }

    public synchronized void setSoftCalib() {
        if(state != ClimbState.SOFT_CALIB) {
            state = ClimbState.SOFT_CALIB;
            mFX.configReverseSoftLimitEnable(false);
            pretimeStamp = Timer.getFPGATimestamp();
            calibrated = false;
        }
    }

    public synchronized void setClimb(double target) {
        if (calibrated && state != ClimbState.CLIMBING) {
            state = ClimbState.CLIMBING;
        }
        Turret.getInstance().setAngle(0, 0);
        mPIO.turret_desiredpos = target;
        if (target < 10000) {
            mFX.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 5, 5, 0.1));
        } else {
            mFX.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 5, 5, 0.1));

        }
    }
    
    public void updateClimbing() {
        if(Math.abs(Turret.getInstance().getTurretDegree())<5){
            mPIO.climb_targetpos = mPIO.turret_desiredpos;
        } else {
            mPIO.climb_targetpos = mPIO.climb_postick;
        }
    }

    public double getClimbPos() {
        return mPIO.climb_postick;
    }

    public double getClimbTarget() {
        return mPIO.climb_targetpos;
    }

    public boolean getClimbCalibrated() {
        return calibrated;
    }

    public boolean isMagicInPosition() {
        return Math.abs(mPIO.climb_postick - mPIO.climb_targetpos) < ALLOWABLE_ERROR;
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        if(Constants.kSuperStructureDebug) {
            SmartDashboard.putString("CLIMB STATE", state.toString());
            SmartDashboard.putNumber("CLIMB POSTICK", mPIO.climb_postick);
            SmartDashboard.putNumber("CLIMB TARGETPOS", mPIO.climb_targetpos);
            SmartDashboard.putNumber("CLIMB CURR", mPIO.climb_curr);
            SmartDashboard.putNumber("CLIMB OUTPERC", mPIO.climb_outperc);
            SmartDashboard.putBoolean("CLIMB CALIB", calibrated);
            SmartDashboard.putNumber("CLIMB VELTICK", mPIO.climb_veltick);
        }
    }
}