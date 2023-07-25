package frc.lib6907.subsystem;

import frc.lib6907.loops.ILooper;

public abstract class Subsystem {
    public void writeToLog() {}

    // Optional design pattern for caching periodic reads to avoid hammering the HAL/CAN.
    public void readPeriodicInputs() {}

    // Optional design pattern for caching periodic writes to avoid hammering the HAL/CAN.
    public void writePeriodicOutputs() {}

    public void registerEnabledLoops(ILooper mEnabledLooper) {} 

    public void zeroSensors() {}

    public abstract void stop();

    public abstract boolean checkSystem();

    public abstract void outputTelemetry();
}