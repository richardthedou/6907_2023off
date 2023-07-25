package frc.lib6907.io;

import edu.wpi.first.wpilibj.Timer;
import frc.lib6907.util.LatchedBoolean;

public class PovReader {

    private static final double DEFAULT_PRESS_THRESHOLD = 0.1;      // s
    private static final double SHORT_PRESS_DT = 0.1;

    private double press_holdtime;
    private int lastkey, lastShortKey, lastkeypos;
    private double lastkeystart, lastkeydura, lastshortstart;
    private boolean onHold;
    private LatchedBoolean onRelease, onPress;

    public PovReader() {
        this(DEFAULT_PRESS_THRESHOLD);
    }

    public PovReader(double PRESS_THRESHOLD) {
        this.press_holdtime = PRESS_THRESHOLD;
        onRelease = new LatchedBoolean();
        onPress = new LatchedBoolean();
    }

    public synchronized void update(int pov) {
        if (onRelease.update(pov < 0) && lastkeydura <= press_holdtime) {
            lastShortKey = lastkeypos;
            lastshortstart = Timer.getFPGATimestamp();
        }
        if (onPress.update(pov >= 0)) {
            lastkeystart = Timer.getFPGATimestamp();
        }
        onHold = (pov >= 0);
        if (onHold) {
            lastkeydura = Timer.getFPGATimestamp() - lastkeystart;
        } else {
            lastkeydura = -1.0;
        }

        lastkey = pov;
        if (pov >= 0) {
            lastkeypos = pov;
        }
    }

    public synchronized int getShortPress() {
        int ret = lastShortKey;
        if (Timer.getFPGATimestamp() - lastshortstart > SHORT_PRESS_DT) {
            lastShortKey = -1;
        }
        return ret;
    }

    public synchronized int getLongPress() {
        if (onHold && lastkeydura > press_holdtime) {
            return lastkey;
        }
        return -1;
    }
    
}
