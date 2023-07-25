package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib6907.geometry.GPose2d;
import frc.lib6907.geometry.GRotation2d;
import frc.lib6907.loops.ILooper;
import frc.lib6907.loops.Loop;
import frc.lib6907.subsystem.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.devices.ControlInput;
import frc.robot.util.Util;

public class Turret extends Subsystem {

    private static final double STARTUP_ANGLE = 0.0;
    private static final double TICK_PER_DEG = 12.5 * 2048 / 360;
    private static final double ZERO_OFFSET = .0;

    //TODO: CHECK THIS
    private static final double 
        TURRET_MIN_POS = TICK_PER_DEG * (-90),
        TURRET_MAX_POS = TICK_PER_DEG * (90);
    
    private static double
        TURRET_KP = 3.0,
        TURRET_KI = 0.0,
        TURRET_KD = 250,
        TURRET_KF = 0.0,
        TURRET_IZONE = 200.0;

    private static final double ALLOWABLE_ERROR = 50.0;
        
    private enum TurretState {
        STOP, POSITION, OPEN_LOOP
    }

    private static Turret mInstance;

    public synchronized static Turret getInstance() {
        if (mInstance == null) {
            mInstance = new Turret();
        }
        return mInstance;
    }
    
    private TalonFX mTurret;
    private PeriodicIO mPeriodicIO;
    private TurretState mTurretState;

    private boolean target_angle_out_of_range = false;

    private Turret() {
        mTurret = new TalonFX(Constants.Turret_ID);
        mTurret.configFactoryDefault();
        Util.configStatusFrame(mTurret);
        mTurret.setInverted(false);
        mTurret.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
        mTurret.configClosedloopRamp(0.1, 10);
        mTurret.config_kP(0, TURRET_KP, 10);
        mTurret.config_kI(0, TURRET_KI, 10);
        mTurret.config_kD(0, TURRET_KD, 10);
        mTurret.config_kF(0, TURRET_KF, 10);
        mTurret.config_IntegralZone(0, TURRET_IZONE, 10);
        // mTurret.configMaxIntegralAccumulator(0, 5000.0, 10);
        mTurret.configPeakOutputForward(0.3, 10);
        mTurret.configPeakOutputReverse(-0.3, 10);
        mTurret.configForwardSoftLimitThreshold(TURRET_MAX_POS, 10);
        mTurret.configForwardSoftLimitEnable(true, 10);
        mTurret.configReverseSoftLimitThreshold(TURRET_MIN_POS, 10);
        mTurret.configReverseSoftLimitEnable(true, 10);

        mTurret.selectProfileSlot(0, 0);
        mTurret.setNeutralMode(NeutralMode.Brake);
        mTurret.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, 10);
        mTurret.configVelocityMeasurementWindow(32, 10);
        mTurret.configAllowableClosedloopError(0, ALLOWABLE_ERROR, 10);
        mTurret.enableVoltageCompensation(true);
        mTurret.configVoltageCompSaturation(12, 10);
        mTurret.configNominalOutputForward(0.001, 10);
        mTurret.configNominalOutputReverse(-0.001, 10);
       


        mTurret.configSupplyCurrentLimit(Constants.Turret_Current_Limit, 10);

        mTurretState = TurretState.STOP;
        mPeriodicIO = new PeriodicIO();
        stop();

        // resetByAbsEncoder();

        resetByMaual();

        SmartDashboard.putNumber("Turret KP", TURRET_KP);        
        SmartDashboard.putNumber("Turret KD", TURRET_KD);
        SmartDashboard.putNumber("Turret KI", TURRET_KI);
        SmartDashboard.putNumber("Turret KF", TURRET_KF);
    }
    
    private static class PeriodicIO {
        //INPUTS
        public double turret_pos, turret_vel, turret_outperc;
        public double turret_pos_error;
        public double turret_ang, turret_fieldang;
        public double time_stamp;
        public double turret_stator, turret_supply;

        //OUTPUTS
        public double turret_pos_demand;
        public double turret_vel_demand;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.turret_pos = mTurret.getSelectedSensorPosition();
        mPeriodicIO.turret_vel = mTurret.getSelectedSensorVelocity();
        mPeriodicIO.turret_pos_error = mPeriodicIO.turret_pos_demand - mPeriodicIO.turret_pos;
        mPeriodicIO.turret_stator = mTurret.getStatorCurrent();
        mPeriodicIO.turret_supply = mTurret.getSupplyCurrent();
        
        mPeriodicIO.turret_outperc = mTurret.getMotorOutputPercent();
        mPeriodicIO.turret_ang = ticksToDegree(mPeriodicIO.turret_pos);
        mPeriodicIO.turret_fieldang = SwerveDrive.getInstance().getHeadingDegree() + mPeriodicIO.turret_ang;
        mPeriodicIO.time_stamp = Timer.getFPGATimestamp();




        double p = SmartDashboard.getNumber("Turret KP", TURRET_KP);
        double i = SmartDashboard.getNumber("Turret KI", TURRET_KI);
        double d = SmartDashboard.getNumber("Turret KD", TURRET_KD);
        double f =SmartDashboard.getNumber("Turret KF", TURRET_KF);

        if(p != TURRET_KP){
            mTurret.config_kP(0, p);
        }

        if(i != TURRET_KI){
            mTurret.config_kI(0, i);

        }
        if(d != TURRET_KD){
            mTurret.config_kD(0, d);

        }
        if(f != TURRET_KF){
            mTurret.config_kF(0, f);

        }
    }

    private void resetByAbsEncoder() {
        double abspos = mTurret.getSensorCollection().getIntegratedSensorAbsolutePosition();
        double relpos = 0;
        if (STARTUP_ANGLE < 0) {
            while (ticksToDegree(relpos) - STARTUP_ANGLE > 2048 / TICK_PER_DEG)
                relpos -= 2048;
            relpos -= (2048 - abspos);
        } else {
            while (STARTUP_ANGLE - ticksToDegree(relpos) > 2048 / TICK_PER_DEG)
                relpos += 2048;
            relpos += abspos;
        }
        mTurret.setSelectedSensorPosition(relpos, 0, 10);

    }
    
    private void resetByMaual() {
        mTurret.setSelectedSensorPosition(0, 0, 10);
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Turret.this) {
                    stop();
                }
            }

            @Override
            public void onLoop(double timestamp) {
                RobotState.getInstance().updateVehicleToTurret(mPeriodicIO.time_stamp, 
                    new GPose2d(Constants.kVehicleToTurret, Rotation2d.fromDegrees(getTurretDegree())));

                synchronized (Turret.this) {
                    switch (mTurretState) {
                        case STOP:
                            break;
                        case POSITION:
                            break;
                        case OPEN_LOOP:
                            break;
                        default:
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
    public synchronized void writePeriodicOutputs() {
        if (mTurretState == TurretState.STOP) {
            mTurret.set(ControlMode.PercentOutput, 0);
        } else if (mTurretState == TurretState.POSITION) {
            mTurret.set(ControlMode.Position, mPeriodicIO.turret_pos_demand, DemandType.ArbitraryFeedForward, TURRET_KF * mPeriodicIO.turret_vel_demand);
        } else if (mTurretState == TurretState.OPEN_LOOP) {
            mTurret.set(ControlMode.PercentOutput, mPeriodicIO.turret_pos_demand);
        } else {
            stop();
            mTurret.set(ControlMode.PercentOutput, 0);
        }
    }

    public synchronized void setAngle(double degree, double degree_vel) {
        degree = Util.placeInAppropriate0To360Scope(0, degree); //-200~160
        double target_ticks = degreeToTicks(degree);
        double target_velticks = degreeToTicks(degree_vel); //ticks per second
        setTicks(target_ticks, target_velticks);
    }

    public synchronized void setFieldAngle(double field_degree, double field_degree_vel) {
        setAngle(field_degree-SwerveDrive.getInstance().getHeadingDegree(), field_degree_vel);; //TODO: tangent movement 
    }

    private synchronized void setTicks(double ticks, double target_velticks) {
        if (mTurretState != TurretState.POSITION) {
            mTurretState = TurretState.POSITION;
        }

        if (ticks < TURRET_MIN_POS) {
            mPeriodicIO.turret_pos_demand = TURRET_MIN_POS;
            mPeriodicIO.turret_vel_demand = 0;
            target_angle_out_of_range = true;

        } else if(ticks > TURRET_MAX_POS) {
            mPeriodicIO.turret_pos_demand = TURRET_MAX_POS;
            mPeriodicIO.turret_vel_demand = 0;
            target_angle_out_of_range = true;
        }else{
            mPeriodicIO.turret_pos_demand = ticks;
            mPeriodicIO.turret_vel_demand = target_velticks;
            target_angle_out_of_range = false;
        }
    }

    public synchronized void setOpenLoop(double perc) {
        if (mTurretState != TurretState.OPEN_LOOP) {
            mTurretState = TurretState.OPEN_LOOP;
        }
        mPeriodicIO.turret_pos_demand = perc;
    }

    @Override
    public synchronized void stop() {
        mTurretState = TurretState.STOP;
        mPeriodicIO.turret_pos_demand = 0.0;
    }

    private static double degreeToTicks(double degree) {
        return degree * TICK_PER_DEG + ZERO_OFFSET;
    }

    private static double ticksToDegree(double ticks) {
        return (ticks - ZERO_OFFSET) / TICK_PER_DEG;
    }

    public synchronized double getTurretDegree() {
        return mPeriodicIO.turret_ang;
    }

    public synchronized GRotation2d getTurretAngle() {
        return GRotation2d.fromDegrees(mPeriodicIO.turret_ang);
    }

    public synchronized double getTurretDegreeFieldRel() {
        return mPeriodicIO.turret_fieldang;
    }

    public boolean isTurretInPosition() {
        return !target_angle_out_of_range && (mTurretState == TurretState.POSITION) && Math.abs(mPeriodicIO.turret_pos_error) < 100.0 ;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }
    
    @Override
    public void outputTelemetry() {
        if (Constants.kSuperStructureDebug) {
            SmartDashboard.putNumber("Turret: Position", mPeriodicIO.turret_pos);
            SmartDashboard.putNumber("Turret: Demand", mPeriodicIO.turret_pos_demand);
            SmartDashboard.putNumber("Turret: Position Error", mPeriodicIO.turret_pos_error);
            SmartDashboard.putNumber("Turret: Output Percentage", mPeriodicIO.turret_outperc);
            SmartDashboard.putNumber("Turret: Velocity", mPeriodicIO.turret_vel);
            SmartDashboard.putNumber("Turret: Stator Current", mPeriodicIO.turret_stator);
            SmartDashboard.putString("Turret: State", mTurretState.toString());
            SmartDashboard.putNumber("Turret: Degree", getTurretDegree());
            SmartDashboard.putNumber("Turret: Supply Current", mPeriodicIO.turret_supply);
            SmartDashboard.putBoolean("Turret: Target Angle Out of Range", target_angle_out_of_range);
        }
        
    }
    
}
