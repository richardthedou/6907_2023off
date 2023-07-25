package frc.lib6907.io;

import edu.wpi.first.wpilibj.Timer;
import frc.lib6907.util.LatchedBoolean;

public class ButtonReader {

    private static final double DEFAULT_PRESS_THRESHOLD = 0.3;      // s
    private static final double SHORT_PRESS_DT = 0.1;

    private double pressHoldTime;
    private LatchedBoolean onPress, onRelease;
    private double lastSpRelTime, lastPressTime;
    private boolean lastState;

    public ButtonReader() {
        this(DEFAULT_PRESS_THRESHOLD);
    }

    public ButtonReader(double holdTime) {
        pressHoldTime = holdTime;
        onPress = new LatchedBoolean();
        onRelease = new LatchedBoolean();
    }

    public synchronized void update(boolean state) {
        double now = Timer.getFPGATimestamp();
        if (onPress.update(state)) {
            lastPressTime = now;
        }
        if (onRelease.update(!state) && (now - lastPressTime < pressHoldTime)) {
            lastSpRelTime = now;
        }
        lastState = state;
    }

    public synchronized boolean getShortPress() {
        return Timer.getFPGATimestamp() - lastSpRelTime < SHORT_PRESS_DT;
    }

    public synchronized boolean getLongPress() {
        return lastState && (Timer.getFPGATimestamp() - lastPressTime > pressHoldTime);
    }
    
}
