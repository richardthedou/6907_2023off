package frc.lib6907.util;

import frc.lib6907.loops.ILooper;
import frc.lib6907.loops.Loop;
import frc.lib6907.loops.Looper;
import frc.lib6907.subsystem.Subsystem;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Used to reset, start, stop, and update all subsystems at once
 */
public class SubsystemManager implements ILooper {
    public static SubsystemManager mInstance = null;

    private List<Subsystem> mAllSubsystems;
    private List<Loop> mLoops = new ArrayList<>();

    private SubsystemManager() {}

    public static SubsystemManager getInstance() {
        if (mInstance == null) {
            mInstance = new SubsystemManager();
        }

        return mInstance;
    }

    public void outputToSmartDashboard() {
        mAllSubsystems.forEach(Subsystem::outputTelemetry);
    }

    public boolean checkSubsystems() {
        boolean ret_val = true;

        for (Subsystem s : mAllSubsystems) {
            ret_val &= s.checkSystem();
        }

        return ret_val;
    }

    public void stop() {
        mAllSubsystems.forEach(Subsystem::stop);
    }

    public List<Subsystem> getSubsystems() {
        return mAllSubsystems;
    }

    public void setSubsystems(Subsystem[]... subsystems) {
        List<Subsystem> allSubsystems = new ArrayList<>();
        for (int i = 0; i < subsystems.length; i++) {
            for (int j = 0; j < subsystems[i].length; j++) {
                allSubsystems.add(subsystems[i][j]);
            }
        }

        mAllSubsystems = allSubsystems;
    }

    public class EnabledLoop implements Loop {
        @Override
        public void onStart(double timestamp) {
            mLoops.forEach(l -> l.onStart(timestamp));
        }

        @Override
        public void onLoop(double timestamp) {
            double s = Timer.getFPGATimestamp();
            mAllSubsystems.forEach(Subsystem::readPeriodicInputs);
            double t1 = Timer.getFPGATimestamp();
            SmartDashboard.putNumber("looper_rin", (t1 - s) * 1000.0);
            mLoops.forEach(l -> l.onLoop(timestamp));
            double t2 = Timer.getFPGATimestamp();
            SmartDashboard.putNumber("looper_loop", (t2 - t1) * 1000.0);
            mAllSubsystems.forEach(Subsystem::writePeriodicOutputs);
            double t3 = Timer.getFPGATimestamp();
            SmartDashboard.putNumber("looper_rout", (t3 - t2) * 1000.0);
        }

        @Override
        public void onStop(double timestamp) {
            mLoops.forEach(l -> l.onStop(timestamp));
        }
    }

    private class DisabledLoop implements Loop {
        @Override
        public void onStart(double timestamp) {}

        @Override
        public void onLoop(double timestamp) {
            mAllSubsystems.forEach(Subsystem::readPeriodicInputs);
            mAllSubsystems.forEach(Subsystem::writePeriodicOutputs);
        }

        @Override
        public void onStop(double timestamp) {}
    }

    public void registerEnabledLoops(Looper enabledLooper) {
        mAllSubsystems.forEach(s -> s.registerEnabledLoops(this));
        enabledLooper.register(new EnabledLoop());
    }

    public void registerDisabledLoops(Looper disabledLooper) {
        disabledLooper.register(new DisabledLoop());
    }

    @Override
    public void register(Loop loop) {
        mLoops.add(loop);
    }
}