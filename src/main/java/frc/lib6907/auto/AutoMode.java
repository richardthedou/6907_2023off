package frc.lib6907.auto;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;

public abstract class AutoMode {

    private ArrayList<Action> mActions;
    private Pose2d startPos;

    public AutoMode(Pose2d robotStart) {
        mActions = new ArrayList<Action>();
        startPos = robotStart;
    }

    protected void addAction(Action a) {
        mActions.add(a);
    }

    public void initAuto() {
        
    }

    public ArrayList<Action> getActionList() {
        return mActions;
    }

    public Pose2d getStartPos() {
        return startPos;
    }

}