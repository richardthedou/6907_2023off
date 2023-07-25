package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib6907.geometry.GPose2d;
import frc.lib6907.geometry.GRotation2d;
import frc.lib6907.geometry.GTranslation2d;
import frc.lib6907.loops.ILooper;
import frc.lib6907.loops.Loop;
import frc.lib6907.subsystem.Subsystem;
import frc.lib6907.util.CrashTrackingRunnable;
import frc.robot.Constants;
import frc.robot.util.Util;

public class SwerveDriveModule extends Subsystem {

    private static final double ROTATE_TPR = 2048.0 * 6.0, DRIVE_TPR = 2048.0 * 28 / 3;

    public static final int DRIVE_MAXV = 20000, ROTATE_MAXV = 14000;

    public static final double DRIVE_MAXV_MS = 4.0, DRIVE_MAXA_MS2 = 4.0;

    private static final double ROTATE_KP = 0.7, ROTATE_KI = 0.0, ROTATE_KD = 1.8, ROTATE_KF = 0.045;
    private double DRIVE_ARB_F;

    private final double HALF_TUQI = 500; // ticks

    // private static final double DRIVE_KP_M = 0.0, DRIVE_KI_M = 0.0, DRIVE_KD_M =
    // 0.0, DRIVE_KF_M = 0.0;

    private TalonFX mDrive;
    private TalonFX mRotate;
    DigitalInput lightGate;
    private SwerveModuleConfigV2 mConfig;

    private double lastDriveDist;
    private double lastRotateAngle;
    private GTranslation2d AbsModulePose;
    private GTranslation2d mEstimatedCenterPosition;

    private static final SupplyCurrentLimitConfiguration drive_supply_limit = new SupplyCurrentLimitConfiguration(true,
            30, 40, 0.01);
    private static final SupplyCurrentLimitConfiguration rotate_supply_limit = new SupplyCurrentLimitConfiguration(true,
            15, 15, 0.01);

    public static class SwerveModuleConfigV2 {
        public int moduleID = 0;
        public int drive_ID = 0;
        public int rotate_ID = 0;
        public int lightGateID = 0;
        public int rotate_Offset = 0;
        public boolean invertDrive = false;
        public boolean invertRotate = false;
        public double DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KF;
        public double ROTATE_KP, ROTATE_KI, ROTATE_KD, ROTATE_KF;
        public GTranslation2d modulePosition = new GTranslation2d();
        public String moduleName = "";
    }

    private PeriodicIO mPeriodicIO;

    private boolean module_zeroed = true;
    private boolean detected = false;
    private boolean last_detected = false;
    private boolean captured = false;
    private double init_status = 0; // 0 not inited, 1 captured tuqi, 2 reached one side,
    private Notifier mNotifier;

    private double initialized_tick = 0;

    public static class PeriodicIO {
        // INPUTS
        public double rotate_pos, rotate_vel, rotate_err;
        public double rotate_outperc, rotate_curr;
        public double rotate_abs_pos;

        public double drive_pos, drive_vel, drive_err;
        public double drive_outperc, drive_curr;

        public double curr_time_stamp;

        // OUTPUTS
        public ControlMode rotate_mode = ControlMode.MotionMagic;
        public ControlMode drive_mode = ControlMode.Velocity;
        public double drive_targetvel, drive_demand, drive_magictarg;
        public double rotate_targetpos, rotate_demand;
    }

    public SwerveDriveModule(SwerveModuleConfigV2 config) {
        mDrive = new TalonFX(config.drive_ID);
        mRotate = new TalonFX(config.rotate_ID);
        mConfig = config;
        mPeriodicIO = new PeriodicIO();
        lightGate = new DigitalInput(config.lightGateID);

        mRotate.configFactoryDefault();
        Util.configStatusFrame(mRotate);
        mRotate.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
        mRotate.configVelocityMeasurementWindow(32, 10);
        mRotate.setInverted(config.invertRotate);
        mRotate.setNeutralMode(NeutralMode.Brake);
        mRotate.configMotionAcceleration((int) (ROTATE_MAXV * 7), 10);
        mRotate.configMotionCruiseVelocity(ROTATE_MAXV, 10);
        mRotate.selectProfileSlot(0, 0);
        mRotate.config_kP(0, ROTATE_KP, 10);
        mRotate.config_kI(0, ROTATE_KI, 10);
        mRotate.config_kD(0, ROTATE_KD, 10);
        mRotate.config_kF(0, ROTATE_KF, 10);

        // mRotate.setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated, 2);
        mRotate.configAllowableClosedloopError(0, 0, 10);
        mRotate.configClosedLoopPeakOutput(0, 0.95, 10);

        mRotate.configSupplyCurrentLimit(rotate_supply_limit);
        mRotate.enableVoltageCompensation(true);
        mRotate.configVoltageCompSaturation(12, 10);

        mRotate.configNeutralDeadband(0.001, 10);
        mRotate.setSelectedSensorPosition(0);
        mRotate.configSupplyCurrentLimit(Constants.SWERVE_MOTOR_SUPPLY_LIMIT, 10);


        mDrive.configFactoryDefault();
        Util.configStatusFrame(mDrive);
        mDrive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
        mDrive.configVelocityMeasurementWindow(32, 10);
        mDrive.setSelectedSensorPosition(0, 0, 10);

        mDrive.setInverted(config.invertDrive);
        mDrive.setNeutralMode(NeutralMode.Brake);
        // vel slot
        mDrive.selectProfileSlot(0, 0);
        mDrive.config_kP(0, config.DRIVE_KP, 10);
        mDrive.config_kI(0, config.DRIVE_KI, 10);
        mDrive.config_kD(0, config.DRIVE_KD, 10);
        mDrive.config_kF(0, config.DRIVE_KF, 10);
        mDrive.configAllowableClosedloopError(0, 0, 10);

        mDrive.configOpenloopRamp(0.3, 10);
        mDrive.configClosedloopRamp(0.5, 10);
        mDrive.configClosedLoopPeakOutput(0, 0.95, 10);

        mDrive.configSupplyCurrentLimit(drive_supply_limit);
        mDrive.enableVoltageCompensation(true);
        mDrive.configVoltageCompSaturation(12, 10);
        mDrive.configNeutralDeadband(0.001, 10);
        mDrive.configSupplyCurrentLimit(Constants.SWERVE_MOTOR_SUPPLY_LIMIT, 10);

        AbsModulePose = new GTranslation2d();

        mNotifier = new Notifier(new CrashTrackingRunnable() { // 单开线程做归零
            @Override
            public synchronized void runCrashTracked() {
                synchronized (this){
                detected = lightGate.get();

                mPeriodicIO.rotate_pos = mRotate.getSelectedSensorPosition();
                
                if (!detected && !last_detected) { // spin to make detected
                    mRotate.set(ControlMode.PercentOutput, -0.10);
                } else if (detected) {
                    mRotate.set(ControlMode.PercentOutput, 0.04);
                } else if (!detected && last_detected){
                    mRotate.set(ControlMode.PercentOutput, 0);
                    module_zeroed = true;
                    mPeriodicIO.rotate_targetpos = 0;
                    mRotate.setSelectedSensorPosition(0, 0, 10);
                    initialized_tick = mPeriodicIO.rotate_abs_pos;
                    mRotate.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 50);
                    mNotifier.stop();
                }
                last_detected = detected;
                }
            }
        });

    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.rotate_curr = mRotate.getStatorCurrent();
        mPeriodicIO.rotate_outperc = mRotate.getMotorOutputPercent();
        mPeriodicIO.rotate_pos = mRotate.getSelectedSensorPosition(0);
        mPeriodicIO.rotate_err = mPeriodicIO.rotate_targetpos - mPeriodicIO.rotate_pos;
        mPeriodicIO.rotate_vel = mRotate.getSelectedSensorVelocity(0);
        mPeriodicIO.rotate_abs_pos = mRotate.getSensorCollection().getIntegratedSensorAbsolutePosition();

        mPeriodicIO.drive_curr = mDrive.getStatorCurrent();
        mPeriodicIO.drive_outperc = mRotate.getMotorOutputPercent();
        mPeriodicIO.drive_pos = mDrive.getSelectedSensorPosition(0);
        mPeriodicIO.drive_vel = mDrive.getSelectedSensorVelocity(0);
        mPeriodicIO.drive_err = mPeriodicIO.drive_targetvel - mPeriodicIO.drive_vel;
        mPeriodicIO.curr_time_stamp = Timer.getFPGATimestamp();
    }


    @Override
    public void registerEnabledLoops(ILooper in) {
        in.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
                synchronized (SwerveDriveModule.this) {
                    // stop();
                }
            }
            
            @Override
            public void onLoop(double timestamp) {
                synchronized (SwerveDriveModule.this) {       

                }
            }

            @Override
            public void onStop(double timestamp) {
                synchronized (SwerveDriveModule.this) {
                    stop();
                }
            }

        });
    }

    public synchronized void writePeriodicOutputs() {
        if(!module_zeroed){
            return;
        } 
        
        if (mPeriodicIO.rotate_mode == ControlMode.PercentOutput)
            mRotate.set(ControlMode.PercentOutput, mPeriodicIO.rotate_demand);
        else if (mPeriodicIO.rotate_mode == ControlMode.MotionMagic)
            mRotate.set(ControlMode.MotionMagic, mPeriodicIO.rotate_targetpos);
        else
            mRotate.set(ControlMode.PercentOutput, 0.0);

        if (mPeriodicIO.drive_mode == ControlMode.PercentOutput)
            mDrive.set(ControlMode.PercentOutput, mPeriodicIO.drive_demand);
        else if (mPeriodicIO.drive_mode == ControlMode.Velocity) {
            if ((int) mPeriodicIO.drive_targetvel != 0) {
                mDrive.set(ControlMode.Velocity, mPeriodicIO.drive_targetvel);
            } else {
                mDrive.set(ControlMode.PercentOutput, 0);
            }
        } else
            mDrive.set(ControlMode.PercentOutput, 0.0);
    }

    public synchronized void setAnglePerc(double perc) {
        mPeriodicIO.rotate_mode = ControlMode.PercentOutput;
        mPeriodicIO.rotate_demand = perc;
    }

    public synchronized void setModuleAngle(double goal) {
        double newAngle = Util.placeInAppropriate0To360Scope(getRawAngle(),
                goal + unitToAngleDegree(mConfig.rotate_Offset));
        mPeriodicIO.rotate_mode = ControlMode.MotionMagic;
        mPeriodicIO.rotate_targetpos = angleDegreeToUnit(newAngle);
    }

    public synchronized void setDrivePerc(double perc) {
        mPeriodicIO.drive_mode = ControlMode.PercentOutput;
        mPeriodicIO.drive_demand = perc;
    }

    public synchronized void setDriveVeltick(double veltick) {
        mPeriodicIO.drive_mode = ControlMode.Velocity;
        mPeriodicIO.drive_targetvel = veltick;
    }

    public double getRawAngle() {
        return unitToAngleDegree(mPeriodicIO.rotate_pos);
    }

    public GTranslation2d getModulePositionToCenter() {
        return mConfig.modulePosition;
    }

    public static double unitToAngleDegree(double d) {
        return d / ROTATE_TPR * 360.0;
    }

    public static int angleDegreeToUnit(double angle) {
        return (int) (angle * ROTATE_TPR / 360.0);
    }

    public static double unitToMetersPerSecond(double drive_calc_vel) {
        return drive_calc_vel / DRIVE_TPR * (Constants.SWERVE_WHEEL_DIAMETER * Math.PI) * 10;
    }

    public static double unitToMeters(double drive_pos) {
        return drive_pos / DRIVE_TPR * (Constants.SWERVE_WHEEL_DIAMETER * Math.PI);
    }

    public Rotation2d getModuleAngle() {
        return Rotation2d.fromDegrees(unitToAngleDegree(mPeriodicIO.rotate_pos - mConfig.rotate_Offset));
        // return GRotation2d.fromDegrees(getRawAngle() -
        // unitToAngleDegree(mConfig.rotate_Offset));
    }

    public synchronized void setDriveVector(GTranslation2d driveVector, boolean isVelMode) {
        // if (driveVector.getNorm() == 0) {
        //     setDrivePerc(0);
        //     return;
        // }
        if (Util.shouldReverse(driveVector.direction().getDegrees(), getModuleAngle().getDegrees())) {
            setModuleAngle(driveVector.direction().getDegrees() + 180.0);
            if (isVelMode)
                setDriveVeltick(-driveVector.getNorm() * SwerveDriveModule.DRIVE_MAXV);
            else
                setDrivePerc(-driveVector.getNorm());
        } else {
            setModuleAngle(driveVector.direction().getDegrees());
            if (isVelMode)
                setDriveVeltick(driveVector.getNorm() * SwerveDriveModule.DRIVE_MAXV);
            else {
                setDrivePerc(driveVector.getNorm());
            }
        }
    }

    /*
    //for 23 and later
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(unitToMeters(mPeriodicIO.drive_pos), getModuleAngle());
    }
    */

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(unitToMetersPerSecond(mPeriodicIO.drive_vel), getModuleAngle());
    }

    public double getUpdateTime() {
        return mPeriodicIO.curr_time_stamp;
    }

    public void resetModulePose(GPose2d centerPose) {
        AbsModulePose = centerPose.getTranslation()
                .translateBy(getModulePositionToCenter().rotateBy(centerPose.getRotation()));
    }

    public void updateModulePosition(GRotation2d centerHeading) {
        double currDriveDist = mDrive.getSelectedSensorPosition();
        double currRotateAngle = mRotate.getSelectedSensorPosition();

        double deltaDist = unitToMetersPerSecond(currDriveDist - lastDriveDist);
        double deltaAngle = unitToAngleDegree(currRotateAngle - lastRotateAngle) / 180.0 * Math.PI;

        if (deltaAngle != 0)
            deltaDist *= 2 * Math.sin(deltaAngle / 2) / deltaAngle;

        GTranslation2d deltaPose = new GTranslation2d(deltaDist,
                getModuleAngle().rotateBy(GRotation2d.fromDegrees(-deltaAngle / 2)).rotateBy(centerHeading)); // robot
                                                                                                              // oriented
                                                                                                              // ->
                                                                                                              // field
                                                                                                              // oriented
        AbsModulePose = AbsModulePose.translateBy(deltaPose);
        mEstimatedCenterPosition = AbsModulePose.minus(getModulePositionToCenter().rotateBy(centerHeading));
        lastDriveDist = currDriveDist;
        lastRotateAngle = currRotateAngle;
    }

    public String getModuleName() {
        return "MODULE " + mConfig.moduleName;
    }

    public GTranslation2d getEstimatedCenterPose() {
        return mEstimatedCenterPosition;
    }

    public synchronized void initialize() {
        module_zeroed = false;
        captured = false;
        detected = false;
        mRotate.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 2);
        mNotifier.startPeriodic(0.002);
    }

    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        outputTelemetry(false);
    }

    public boolean getInitialized() {
        return module_zeroed;
    }

    // @Override
    public void outputTelemetry(boolean forceDebug) {
        if (SmartDashboard.getBoolean("Swerve Debug", Constants.kSwerveDebug)) {
            SmartDashboard.putNumber(getModuleName() + " rotate_err", mPeriodicIO.rotate_err);
            SmartDashboard.putNumber(getModuleName() + " rotate_targetpos", mPeriodicIO.rotate_targetpos);
            SmartDashboard.putNumber(getModuleName() + " rotate_pos", mPeriodicIO.rotate_pos);
            SmartDashboard.putNumber(getModuleName() + " rotate_abso_pos", mPeriodicIO.rotate_abs_pos);
            SmartDashboard.putNumber(getModuleName() + " rotate_outperc", mPeriodicIO.rotate_outperc);
            SmartDashboard.putNumber(getModuleName() + " drive current", mDrive.getSupplyCurrent());
            SmartDashboard.putBoolean(getModuleName() + " module zeroed", module_zeroed);
            SmartDashboard.putString(getModuleName() + " Current Angle", getModuleAngle().toString());

            SmartDashboard.putNumber(getModuleName() + " drive_err",
                    mPeriodicIO.drive_err);
            SmartDashboard.putNumber(getModuleName() + " drive_targetvel",
                    mPeriodicIO.drive_targetvel);
            SmartDashboard.putNumber(getModuleName() + " drive_vel",
                    mPeriodicIO.drive_vel);

            SmartDashboard.putBoolean(getModuleName() + " lightgate detected", lightGate.get());

        }

    }

    public synchronized void stop() {
        mPeriodicIO.drive_mode = ControlMode.PercentOutput;
        mPeriodicIO.drive_demand = 0.0;
        mPeriodicIO.rotate_mode = ControlMode.PercentOutput;
        mPeriodicIO.rotate_demand = 0.0;
    }

    
	public TalonFX[] getFalcons(){
		return new TalonFX[]{mRotate, mDrive};
	}

}
