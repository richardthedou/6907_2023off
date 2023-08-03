package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib6907.loops.ILooper;
import frc.lib6907.loops.Loop;
import frc.robot.Constants;
import frc.robot.util.Util;
import frc.lib6907.subsystem.Subsystem;

public class Shooter extends Subsystem {

    private static final int VELRANGE_MAXCNT = 5;
    private static final double VELRANGE_THRESHOLD = 0.3;
    private static final double SPIN_PERC = 0.95;
    private static final double IDLE_PERC = 0.0; 
    public static final double MAX_VELADJ_PERC = 0.04;
    public static final double ALLOWABLE_ERROR = 150.0;

    private static final double VEL_kP = 8.0,
            VEL_kI = 0.02,
            VEL_kD = 1000,
            VEL_kF = 0.0492;
    private static final double CLOSE_SPIN_RAMP = 0.5, // >= 1.5
            CLOSE_HOLD_RAMP = 0.15;

    public enum ShooterState {
        STOP,
        IDLE, //DEFAULT
        SHOOT_SPINNING,
        SHOOT_VELOCITY,
        Reverse
    }

    private TalonFX mShooterLeft, mShooterRight;
    private ShooterState mShooterState;
    private PeriodicIO mPeriodicIO;
    private boolean spinRamp;
    private int velRangeCnt;

    private static Shooter sInstance;

    public static Shooter getInstance() {
        if (sInstance == null)
            sInstance = new Shooter();
        return sInstance;
    }

    private Shooter() {
        mShooterState = ShooterState.STOP;
        mPeriodicIO = new PeriodicIO();
        spinRamp = true;
        velRangeCnt = 0;

        mShooterRight = new TalonFX(Constants.Shooter_Right_ID);
        mShooterLeft = new TalonFX(Constants.Shooter_Left_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.openloopRamp = CLOSE_SPIN_RAMP;
        config.closedloopRamp = CLOSE_SPIN_RAMP;
        config.slot0.kP = VEL_kP;
        config.slot0.kI = VEL_kI;
        config.slot0.kD = VEL_kD;
        config.slot0.kF = VEL_kF;
        config.peakOutputForward = 0.95;
        config.peakOutputReverse = -0.95;
        config.statorCurrLimit.enable = true;
        config.statorCurrLimit.currentLimit = 40.0;
        config.statorCurrLimit.triggerThresholdCurrent = 40.0;
        config.statorCurrLimit.triggerThresholdTime = 2.0;

        mShooterRight.configAllSettings(config);
        mShooterLeft.configAllSettings(config);
        mShooterRight.config_IntegralZone(0, 300, 10);
        mShooterLeft.config_IntegralZone(0, 300, 10);

        // Util.configStatusFrame(mShooterRight);
        // Util.configStatusFrameSlave(mShooterLeft);
        mShooterRight.selectProfileSlot(0, 0);
        mShooterLeft.selectProfileSlot(0, 0);
        mShooterRight.setNeutralMode(NeutralMode.Coast);
        mShooterLeft.setNeutralMode(NeutralMode.Coast);
        mShooterRight.setInverted(false);
        mShooterLeft.follow(mShooterRight);
        mShooterLeft.setInverted(InvertType.OpposeMaster);
        mShooterRight.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, 10);
        mShooterRight.configVelocityMeasurementWindow(32, 10);
        mShooterLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, 10);
        mShooterLeft.configVelocityMeasurementWindow(32, 10);
        mShooterLeft.enableVoltageCompensation(true);
        mShooterRight.enableVoltageCompensation(true);
        mShooterLeft.configVoltageCompSaturation(12);
        mShooterRight.configVoltageCompSaturation(12);

        
        mShooterRight.configSupplyCurrentLimit(Constants.Shooter_Current_Limit, 10);
        mShooterLeft.configSupplyCurrentLimit(Constants.Shooter_Current_Limit, 10);
    }

    public static class PeriodicIO {
        // INPUTS
        public double shooter_current_left, shooter_current_right;
        public double shooter_temp_left, shooter_temp_right;
        public double shooter_veltick, shooter_outperc;
        public double manual_veladjperc;
        public double shooter_vel_err;

        // OUTPUTS
        public double shooter_targetvel;
    }

    @Override
    public synchronized void readPeriodicInputs() {
     
        mPeriodicIO.shooter_temp_left = Double.NaN;
        mPeriodicIO.shooter_temp_right = Double.NaN;
        mPeriodicIO.shooter_outperc = Double.NaN;

        mPeriodicIO.shooter_current_left = mShooterLeft.getStatorCurrent();
        mPeriodicIO.shooter_current_right = mShooterRight.getStatorCurrent();
        mPeriodicIO.shooter_veltick = mShooterRight.getSelectedSensorVelocity(0);
        mPeriodicIO.shooter_vel_err = mPeriodicIO.shooter_targetvel - mPeriodicIO.shooter_veltick;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        double veldemand = (1.0 + mPeriodicIO.manual_veladjperc) * mPeriodicIO.shooter_targetvel;
        if (mShooterState == ShooterState.SHOOT_SPINNING)
            mShooterRight.set(ControlMode.PercentOutput, SPIN_PERC);
        else if (mShooterState == ShooterState.SHOOT_VELOCITY)
            mShooterRight.set(ControlMode.Velocity, veldemand);
        else if (mShooterState == ShooterState.IDLE) {
            mShooterRight.set(ControlMode.PercentOutput, IDLE_PERC);
        }else if(mShooterState == ShooterState.Reverse){
            mShooterRight.set(ControlMode.PercentOutput, -0.15);
        }else{
            mShooterRight.set(ControlMode.PercentOutput, 0.0);
        }
    }

    @Override
    public void registerEnabledLoops(ILooper in) {
        in.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
                synchronized (Shooter.this) {
                    stop();
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Shooter.this) {
                    switch (mShooterState) {
                        case STOP:
                            break;
                        case SHOOT_SPINNING:
                            updateSpinning(timestamp);
                            break;
                        case SHOOT_VELOCITY:
                            break;
                        case IDLE:
                            break;
                        default:
                            System.out.println("SHOOTER : UNEXPECTED STATE - " + mShooterState);
                            mShooterState = ShooterState.STOP;
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

    private void updateSpinning(double timestamp) {
        if (mPeriodicIO.shooter_veltick > VELRANGE_THRESHOLD * mPeriodicIO.shooter_targetvel) {
            velRangeCnt++;
            if (velRangeCnt > VELRANGE_MAXCNT)
                goShoot();
        } else {
            // CONTINOUS OR NOT
            velRangeCnt = 0;
        }
    }

    private void goShoot() {
        setRamp(false);
        mShooterState = ShooterState.SHOOT_VELOCITY;
    }

    public synchronized void setShootVel(double veltick) {
        if (mShooterState != ShooterState.STOP && mShooterState != ShooterState.IDLE) {
            // update while vt is moving a little bit
            mPeriodicIO.shooter_targetvel = veltick;
            return;
        }

        mPeriodicIO.shooter_targetvel = veltick;
        mShooterState = ShooterState.SHOOT_SPINNING;
        setRamp(true);
        velRangeCnt = 0;
    }

    public synchronized void stop() {
        if (mShooterState == ShooterState.STOP)
            return;

        mPeriodicIO.shooter_targetvel = 0.0;
        mShooterState = ShooterState.STOP;
        setRamp(true);
    }

    public synchronized void setIdle() {
        if (mShooterState != ShooterState.IDLE) {
            mShooterState = ShooterState.IDLE;
        }
        setRamp(true);
    }

    public synchronized void setReverse() {
        if (mShooterState != ShooterState.IDLE) {
            mShooterState = ShooterState.IDLE;
        }
        setRamp(true);
    }

    public synchronized void setVelAdjPerc(double perc) {
        mPeriodicIO.manual_veladjperc = Util.limit(perc, MAX_VELADJ_PERC);
    }

    public synchronized void setRamp(boolean spinRamp) {
        if (this.spinRamp != spinRamp) {
            this.spinRamp = spinRamp;
            if (this.spinRamp) {
                mShooterLeft.configOpenloopRamp(CLOSE_SPIN_RAMP, 10);
                mShooterRight.configOpenloopRamp(CLOSE_SPIN_RAMP, 10);
                mShooterLeft.configClosedloopRamp(CLOSE_SPIN_RAMP, 10);
                mShooterRight.configClosedloopRamp(CLOSE_SPIN_RAMP, 10);
            } else {
                mShooterLeft.configOpenloopRamp(CLOSE_HOLD_RAMP, 10);
                mShooterLeft.configOpenloopRamp(CLOSE_HOLD_RAMP, 10);
                mShooterRight.configClosedloopRamp(CLOSE_HOLD_RAMP, 10);
                mShooterRight.configClosedloopRamp(CLOSE_HOLD_RAMP, 10);
            }
        }
    }



    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        if (Constants.kSuperStructureDebug) {
            
            SmartDashboard.putNumber("SHOOTER PERC", mPeriodicIO.shooter_outperc);
            SmartDashboard.putNumber("SHOOTER VEL ERR", mPeriodicIO.shooter_vel_err);
            SmartDashboard.putNumber("SHOOTER VELTICK", mPeriodicIO.shooter_veltick);
            SmartDashboard.putNumber("SHOOTER LEFT CURRENT", mPeriodicIO.shooter_current_left);
            SmartDashboard.putNumber("SHOOTER LEFT TEMP", mPeriodicIO.shooter_temp_left);
            SmartDashboard.putNumber("SHOOTER RIGHT CURRENT", mPeriodicIO.shooter_current_right);
            SmartDashboard.putNumber("SHOOTER RIGHT TEMP", mPeriodicIO.shooter_temp_right);
            SmartDashboard.putString("SHOOTER STATE", mShooterState.toString());

            SmartDashboard.putNumber("Shooter: Target Veltick", mPeriodicIO.shooter_targetvel);
        }
    }

    public ShooterState getShooterState() {
        return mShooterState;
    }

    public double getSetpoint() {
        return mPeriodicIO.shooter_targetvel;
    }

    public double getTargetSpeed() {
        return mPeriodicIO.shooter_veltick;
    }

    public boolean getVelocityReached() {
        return Math.abs(mPeriodicIO.shooter_vel_err) < ALLOWABLE_ERROR && mPeriodicIO.shooter_veltick > 4000;
    }

    public double getSpinningVeltick() {
        return mPeriodicIO.shooter_veltick;
    }

    public double getSpinningPercentage() {
        return mPeriodicIO.shooter_veltick / mPeriodicIO.shooter_targetvel;
    }

}

