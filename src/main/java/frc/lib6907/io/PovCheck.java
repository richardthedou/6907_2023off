package frc.lib6907.io;

public class PovCheck {
    
    private ButtonCheck mPovState;
    private int lastPosValue;
    private boolean longPressing;

    public PovCheck() {
        mPovState = new ButtonCheck();
        lastPosValue = -1;
        longPressing = false;
    }

    public synchronized void update(int pov) {
        if (pov >= 0) lastPosValue = pov;
        mPovState.update(pov >= 0);
        if (mPovState.longPressed()) longPressing = true;
        if (mPovState.longReleased()) longPressing = false;
    }

    public synchronized int shortReleased() {
        return mPovState.shortReleased() ? lastPosValue : -1;
    }

    public synchronized int longHeld() {
        return (longPressing) ? lastPosValue : -1;
    }

}