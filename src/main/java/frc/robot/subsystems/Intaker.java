package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib6907.loops.ILooper;
import frc.lib6907.loops.Loop;
import frc.lib6907.subsystem.Subsystem;
import frc.robot.Constants;
import frc.robot.util.Util;

public class Intaker extends Subsystem {
    public final double EXTEND_TICK = 4800;
    public final double HOME_TICK = 1000;
    public final double PLATE_DOWN_TICK = 0;
    public final double PLATE_FEED_TICK = 3000;
    public final double PLATE_SHOOT_TICK = 14200;
    public final double PLATE_UP_TICK = 11250;
    private final double ZERO_TIME = 0.5; // 复位堵转时间;
    private final double SHOOT_VELTICK = 10000;

    public double lastTranferFullTime = Double.NEGATIVE_INFINITY;
    public boolean lastTransferFull = false;

    public double lastNotShootingTime = Double.POSITIVE_INFINITY;

    public boolean offload_spin = false;

    public enum IntakerState {
        HOME, INTAKE, FEED, HOLDBALL, OUTTAKE
    }

    private boolean extend_initialized = true;
    private boolean hood_initialized = true;
    private double lastExtendRetractTimestamp = Double.POSITIVE_INFINITY;

    private IntakerState mIntakerState;

    private static Intaker mInstance;

    public static Intaker getInstance() {
        if (mInstance == null) {
            mInstance = new Intaker();
        }
        return mInstance;
    }

    private TalonFX mIntakeExtend;
    private TalonFX mIntakeRoller;

    private PeriodicIO mPeriodicIO;

    private Intaker() {
        mPeriodicIO = new PeriodicIO();
        mIntakeExtend = new TalonFX(Constants.Intaker_Extend_ID);
        mIntakeRoller = new TalonFX(Constants.Intaker_Roller_ID);

        mIntakeExtend.configFactoryDefault();
        mIntakeRoller.configFactoryDefault();

        mIntakeExtend.setNeutralMode(NeutralMode.Brake);
        mIntakeRoller.setNeutralMode(NeutralMode.Coast);

        Util.configStatusFrame(mIntakeRoller);
        Util.configStatusFrame(mIntakeExtend);

        mIntakeExtend.setInverted(false);
        mIntakeRoller.setInverted(false);

        // mIntakeExtend.configSupplyCurrentLimit(Constants.Intaker_Current_Limit);
        mIntakeRoller.configSupplyCurrentLimit(Constants.Intaker_Current_Limit);

        mIntakeExtend.configStatorCurrentLimit(Constants.Intaker_Extend_Limit);

        mIntakeExtend.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        mIntakeExtend.setSelectedSensorPosition(0, 0, 10);
        mIntakeExtend.configOpenloopRamp(0.1, 10);
        mIntakeExtend.setNeutralMode(NeutralMode.Brake);

        mIntakeExtend.selectProfileSlot(0, 0);
        mIntakeExtend.config_kP(0, 0.3, 10);
        mIntakeExtend.config_kI(0, 0.0, 10);
        mIntakeExtend.config_kD(0, 3.0, 10);
        mIntakeExtend.config_kF(0, 0.09, 10);
        mIntakeExtend.configClosedLoopPeakOutput(0, 0.15, 10);
        mIntakeExtend.configAllowableClosedloopError(0, 0, 10);
        mIntakeExtend.configMotionCruiseVelocity(3000, 10);
        mIntakeExtend.configMotionAcceleration(3000, 10);

        mIntakeExtend.configNeutralDeadband(0.001);

        mIntakeExtend.configForwardSoftLimitEnable(true, 10);
        mIntakeExtend.configForwardSoftLimitThreshold(6000, 10);

        mIntakeRoller.selectProfileSlot(0, 0);
        mIntakeRoller.config_kP(0, 0.1, 10);
        mIntakeRoller.config_kI(0, 0.0, 10);
        mIntakeRoller.config_kD(0, 0.0, 10);
        mIntakeRoller.config_kF(0, 0.05, 10);
        mIntakeRoller.configClosedLoopPeakOutput(0, 0.95, 10);
        mIntakeRoller.configClosedloopRamp(0.5, 10);
        mIntakeRoller.configAllowableClosedloopError(0, 200, 10);
        mIntakeRoller.setNeutralMode(NeutralMode.Coast);
        mIntakeRoller.configForwardSoftLimitEnable(false, 10);

        

        mIntakerState = IntakerState.HOME;

        mIntakeExtend.configReverseSoftLimitEnable(false, 10);

        extend_initialized = true;
        mIntakeExtend.setSelectedSensorPosition(0);
        mIntakeExtend.configReverseSoftLimitEnable(true, 10);
        mIntakeExtend.configReverseSoftLimitThreshold(100, 10);

        stop();
    }

    public static class PeriodicIO {
        // INPUTS
        public double extend_pos;
        public double extend_vel;
        public double roller_vel;
        public double extend_curr;

        // OUTPUTS
        public double roller_demand, extend_demand;
        public ControlMode roller_mode = ControlMode.PercentOutput, extend_mode = ControlMode.PercentOutput;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        
        mPeriodicIO.extend_pos = mIntakeExtend.getSelectedSensorPosition();
        mPeriodicIO.extend_vel = mIntakeExtend.getSelectedSensorVelocity();
        mPeriodicIO.extend_curr = mIntakeExtend.getStatorCurrent();
        mPeriodicIO.roller_vel = mIntakeRoller.getSelectedSensorVelocity();
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
                synchronized (Intaker.this) {
                    stop();
                    extend_initialized = true;
                    lastExtendRetractTimestamp = timestamp;
                    
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Intaker.this) {
                    if (!extend_initialized) {
                        if (mPeriodicIO.extend_vel < -50) {
                            lastExtendRetractTimestamp = timestamp;
                        }
                        if (mPeriodicIO.extend_vel > -50 && timestamp - lastExtendRetractTimestamp > ZERO_TIME) {
                            extend_initialized = true;
                            mIntakeExtend.setSelectedSensorPosition(0);
                            mIntakeExtend.configReverseSoftLimitEnable(true, 10);
                            mIntakeExtend.configReverseSoftLimitThreshold(100, 10);
                        }
                    }

                    calcDemand();
                }
            }

            @Override
            public void onStop(double timestamp) {
                synchronized (Intaker.this) {
                    stop();
                }
            }

        });
    }

    private void calcDemand() {
        switch (mIntakerState) {
            case FEED:
                mIntakeExtend.configMotionCruiseVelocity(3000);
                mPeriodicIO.extend_mode = ControlMode.MotionMagic;
                mPeriodicIO.extend_demand = 2000 + 1500*Math.sin(2*Math.PI*Timer.getFPGATimestamp());
                mPeriodicIO.roller_mode = ControlMode.PercentOutput;
                mPeriodicIO.roller_demand = 0.0;
                break;
            case HOME:
                mIntakeExtend.configMotionCruiseVelocity(3000);
                mPeriodicIO.extend_mode = ControlMode.MotionMagic;
                mPeriodicIO.extend_demand = 0;
                if(mPeriodicIO.extend_pos > 2000){
                    mPeriodicIO.roller_demand = 0.05;
                }else{
                    mPeriodicIO.roller_demand = 0.0;
                }
                mPeriodicIO.roller_mode = ControlMode.PercentOutput;
                break;
            case INTAKE:
                mIntakeExtend.configMotionCruiseVelocity(3000);
                mPeriodicIO.extend_mode = ControlMode.MotionMagic;
                mPeriodicIO.extend_demand = EXTEND_TICK;
                mPeriodicIO.roller_mode = ControlMode.PercentOutput;
                // mPeriodicIO.roller_demand = mPeriodicIO.extend_pos > 2000? 0.20:0;
                mPeriodicIO.roller_demand = 0.27;

                break;
            case HOLDBALL:
                mPeriodicIO.extend_mode = ControlMode.MotionMagic;
                mPeriodicIO.extend_demand = EXTEND_TICK;
                mPeriodicIO.roller_mode = ControlMode.PercentOutput;
                mPeriodicIO.roller_demand = 0;
            case OUTTAKE:
                mPeriodicIO.extend_mode = ControlMode.MotionMagic;
                mPeriodicIO.extend_demand = EXTEND_TICK + 1000;
                mPeriodicIO.roller_mode = ControlMode.PercentOutput;
                mPeriodicIO.roller_demand = -0.25;
            default:
                break;

        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (!extend_initialized || !hood_initialized) {
            mIntakeExtend.set(ControlMode.PercentOutput, extend_initialized ? 0 : -0.11);
            mIntakeRoller.set(ControlMode.PercentOutput, 0);
        } else {
            mIntakeExtend.set(mPeriodicIO.extend_mode, mPeriodicIO.extend_demand);
            mIntakeRoller.set(mPeriodicIO.roller_mode, mPeriodicIO.roller_demand);
        }
    }



    public synchronized void setIntake() {
        if (mIntakerState != IntakerState.INTAKE) {
            mIntakerState = IntakerState.INTAKE;
        }

    }

    public synchronized void setHoldBall() {
        if (mIntakerState != IntakerState.HOLDBALL) {
            mIntakerState = IntakerState.HOLDBALL;
        }
    }

    public synchronized void setOuttake() {
        if (mIntakerState != IntakerState.OUTTAKE) {
            mIntakerState = IntakerState.OUTTAKE;
        }
    }

    public synchronized void setHome() {
        if (mIntakerState != IntakerState.HOME) {
            mIntakerState = IntakerState.HOME;
        }

    }

    public IntakerState getState() {
        return mIntakerState;
    }

    @Override
    public void stop() {
        setHome();
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {

        if (SmartDashboard.getBoolean("Intake Debug", Constants.kIntakerDebug)) {
            SmartDashboard.putString("Intaker: State", mIntakerState.toString());
            SmartDashboard.putNumber("Intaker: Extend Demand", mPeriodicIO.extend_demand);
            SmartDashboard.putNumber("Intaker: Roller velocity", mPeriodicIO.roller_vel);
            SmartDashboard.putNumber("Intaker: Roller demand", mPeriodicIO.roller_demand);


            SmartDashboard.putNumber("Intaker: Extend position", mPeriodicIO.extend_pos);
            SmartDashboard.putNumber("Intaker: Extend target", mPeriodicIO.extend_demand);
            SmartDashboard.putNumber("Intaker: Spinnig Perc", getSpinningPercentage());

            SmartDashboard.putNumber("Intaker: Extend velocity", mPeriodicIO.extend_vel);

            SmartDashboard.putNumber("Intaker: Extend Current", mIntakeExtend.getSupplyCurrent());

            SmartDashboard.putBoolean("Intaker: Initialized", extend_initialized && hood_initialized);
        }
    }

    public double getSpinningPercentage() {
        return Math.max(0, mPeriodicIO.roller_vel / SHOOT_VELTICK);
    }
}
