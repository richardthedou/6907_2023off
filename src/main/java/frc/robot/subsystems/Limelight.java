package frc.robot.subsystems;


import java.util.List;

import javax.management.MBeanException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib6907.geometry.GTranslation2d;
import frc.lib6907.loops.ILooper;
import frc.lib6907.loops.Loop;
import frc.lib6907.subsystem.Subsystem;
import frc.lib6907.util.vision.TargetInfo;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.Transfer.TransferState;

public class Limelight extends Subsystem {

    private final NetworkTable mNetworkTable;

    private static Limelight theOnlyLL;
    public static Limelight getTheOnlyLL()
    {
        if (theOnlyLL == null) {
            theOnlyLL = new Limelight();
        }
        return theOnlyLL;
    }

    private Limelight() {
        mNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public static class PeriodicIO {
        // INPUTS
        public double latency; // ms
        public boolean hasTarget;
        public boolean confidence;
        public GTranslation2d estCenter = new GTranslation2d();
        public GTranslation2d lastCenter = new GTranslation2d();
        public double timestamp;

        // OUTPUTS
        public int ledMode = LedMode.ON.ordinal();
        public boolean snapshot = false;
    }

    private final PeriodicIO mPeriodicIO = new PeriodicIO();
    private boolean mOutputChanged = true;
    private static final double[] mZeroArray = new double[] { 0.0 };

    private static final double FOVX = 59.6, FOVY = 49.7, PITCH = 30;


    private Translation2d getFieldProj(Translation2d pt, double heigh_diff){
        // double y_p = 1.0 - 2 * pt.getX();
        // double z_p = 1.0 - 2 * pt.getY();

        double x = 1.0;
        double y = Math.tan(Math.toRadians(pt.getX()));
        double z = Math.tan(Math.toRadians(pt.getY()));

        // SmartDashboard.putString("LL: Processing coord", x + " " + y + " " + z);

        Translation2d xz = new Translation2d(x,z);

        //compensate pitch
        xz = xz.rotateBy(Rotation2d.fromDegrees(PITCH));
        x = xz.getX();
        z = xz.getY();

        double scaling = heigh_diff / z;

        SmartDashboard.putNumber("LL scalin", scaling);
        // distance = math.hypot(x, y) * scaling
        Translation2d ret = new Translation2d(x,-y).times(scaling);
        return ret;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.latency = (mNetworkTable.getEntry("tl").getDouble(0.0) + 11) / 1000.0;

        double a = mNetworkTable.getEntry("ta").getDouble(Double.NaN);
        if (a == 0.0) {
            return;
        }
        double x = mNetworkTable.getEntry("tx").getDouble(Double.NaN);
        double y = mNetworkTable.getEntry("ty").getDouble(Double.NaN);

        
        mPeriodicIO.estCenter = new GTranslation2d(getFieldProj(new Translation2d(x, y), 2.62 - 0.7));
        if (!mPeriodicIO.estCenter.equals(mPeriodicIO.lastCenter)) {
            TargetInfo new_update = new TargetInfo(new Pose2d(mPeriodicIO.lastCenter, new Rotation2d()), 0);
            RobotState.getInstance().addVisionUpdate(
                    mPeriodicIO.timestamp - mPeriodicIO.latency,
                    List.of(new_update));
        }
        mPeriodicIO.lastCenter = mPeriodicIO.estCenter;
        

        /* find center parse
        double[] llpy = mNetworkTable.getEntry("llpython").getDoubleArray(mZeroArray);
        mPeriodicIO.hasTarget = (llpy.length == 3);
        if (mPeriodicIO.hasTarget) {
            mPeriodicIO.estCenter = (new GTranslation2d(llpy[0], llpy[1]));
            if (!mPeriodicIO.estCenter.equals(mPeriodicIO.lastCenter)) {
                TargetInfo new_update = new TargetInfo(new Pose2d(mPeriodicIO.lastCenter, new Rotation2d()), 0);
                RobotState.getInstance().addVisionUpdate(
                        mPeriodicIO.timestamp - mPeriodicIO.latency,
                        List.of(new_update));
            }
            mPeriodicIO.lastCenter = mPeriodicIO.estCenter;
        }
        */
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mOutputChanged) {
            mOutputChanged = false;
            mNetworkTable.getEntry("ledMode").setNumber(mPeriodicIO.ledMode);
            mNetworkTable.getEntry("snapshot").setNumber(mPeriodicIO.snapshot ? 1 : 0);
        }
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
                RobotState.getInstance().resetVision();
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Limelight.this) {
                    if (mPeriodicIO.hasTarget) {
                        
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
    public synchronized void outputTelemetry() {
        if (true) {
            SmartDashboard.putNumber("LL: Latency (ms)", mPeriodicIO.latency);
            SmartDashboard.putNumber("LL: Total Latency (s)",
                    mPeriodicIO.latency);
            SmartDashboard.putBoolean("LL: Has Target", mPeriodicIO.hasTarget);
            // SmartDashboard.putBoolean("LL: Result good", mPeriodicIO.confidence);
            SmartDashboard.putString("LL: Center", mPeriodicIO.estCenter.toString());
        }
    }

    public enum LedMode {
        PIPELINE, OFF, BLINK, ON
    }

    public synchronized void setLed(LedMode mode) {
        if (mode.ordinal() != mPeriodicIO.ledMode) {
            mPeriodicIO.ledMode = mode.ordinal();
            mOutputChanged = true;
        }
    }

    public synchronized void setSnapshot(boolean takeSnapshot) {
        if (takeSnapshot ^ mPeriodicIO.snapshot) {
            mPeriodicIO.snapshot = takeSnapshot;
            mOutputChanged = true;
        }
    }

    @Override
    public void stop() {
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

}
