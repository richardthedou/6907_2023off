package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib6907.geometry.GPose2d;
import frc.lib6907.loops.ILooper;
import frc.lib6907.loops.Loop;
import frc.lib6907.subsystem.Subsystem;
import frc.lib6907.util.vision.TargetInfo;
import frc.robot.Robot;
import frc.robot.RobotState;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

/**
 * Subsystem for interacting with the Limelight 2
 */
public class VisionProcessor extends Subsystem {

    private final NetworkTable mNetworkTable;

    private final GPose2d RobotToLens = new GPose2d(0,0,new Rotation2d());


    private static VisionProcessor mInstance;
    public static VisionProcessor getInstance(){
        if(mInstance == null){
            mInstance = new VisionProcessor("Vision");
        }
        return mInstance;
    }

    public VisionProcessor(String tableName) {
        mNetworkTable = NetworkTableInstance.getDefault().getTable(tableName);
    }

    public static class PeriodicIO {
        // INPUTS
        public double latency;
        public double[] raw_input;
        public List<TargetInfo> targets;
    }

    private final PeriodicIO mPeriodicIO = new PeriodicIO();
    private final List<TargetInfo> mTargets = new ArrayList<>();
    private int numTarget = 0;
    
    public Pose2d getRobotToLens(){
        return RobotToLens;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        // mPeriodicIO.latency = mNetworkTable.getEntry("tl").getDouble(0) / 1000.0;
        mPeriodicIO.raw_input = mNetworkTable.getEntry("tags").getDoubleArray(new double[1]);
        numTarget = (int)(double)(mPeriodicIO.raw_input[0]);
        mPeriodicIO.targets = new ArrayList<>();
        //TODO: read targets
        // TargetInfo target = new TargetInfo(x, y, );
        // mPeriodicIO.targets.add(target);

        
    }

    @Override
    public synchronized void writePeriodicOutputs() {
       
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        Loop mLoop = new Loop() {
            @Override
            public void onStart(double timestamp) {
                RobotState.getInstance().resetVision();
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (VisionProcessor.this) {
                    RobotState.getInstance().addVisionUpdate(
                            timestamp - getLatency(),
                            getTarget());
                }

            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        };
        mEnabledLooper.register(mLoop);
    }

    @Override
    public void stop() {
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putNumber("VP: Number of Targets", numTarget);
        SmartDashboard.putNumber("VP: Pipeline Latency (ms)", mPeriodicIO.latency);
    }

    public synchronized List<TargetInfo> getTarget() {
        if (numTarget != 0 && mPeriodicIO.targets != null) {
            return mPeriodicIO.targets;
        }

        return null;
    }

    public double getLatency() {
        return mPeriodicIO.latency + 0.06;
    }


}