package frc.lib6907.auto;

import java.util.ArrayList;

public class AutoRunner {

    private double startTime, lastdt, dt;
    private ArrayList<Action> mActions;

    public AutoRunner(ArrayList<Action> actions) {
        mActions = actions;
    }

    public void initAuto(double start_) {
        startTime = start_;
        lastdt = 0.0;
        dt = 0.0;
    }

    public void updatePeriodic(double timestamp) {
        dt = timestamp-startTime;

        for(Action a : mActions)
            handleAction(a);

        lastdt = dt;
    }

    private void handleAction(Action a) {
        if (dt<a.getStartTime())
            return;
        else if (dt>=a.getStartTime() && lastdt<=a.getStartTime())
            a.start();
        else if (lastdt>a.getStartTime() && dt<a.getEndTime())
            a.run();
        else if (dt>=a.getEndTime() && lastdt<=a.getEndTime())
            a.end();
        else if (lastdt>a.getEndTime())
            return;
    }

}