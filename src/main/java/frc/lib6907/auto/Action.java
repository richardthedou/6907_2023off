package frc.lib6907.auto;

public abstract class Action {

    private double startTime, endTime;

    public Action(double start, double end) {
        startTime = start;
        endTime = end;
    }

    public double getStartTime() { return startTime; }
    
    public double getEndTime() { return endTime; }

    public abstract void start();

    public abstract void run();

    public abstract void end();

}