package frc.lib6907.devices;

import edu.wpi.first.wpilibj.util.Color;
import frc.lib6907.util.CrashTrackingRunnable;
// import frc.robot.util.RPColor;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.GainFactor;

public class ColorSensor {
    // 0 - 2047, bigger closer
    private static final int PROX_CLOSE_BOUND = 200;
    private static final double CONF_BARRIER = 0.92;
    private static final int CONFIDENT_MINCOUNT = 2;

    private final ColorSensorV3 m_colorSensor;
    private final ColorMatch m_colorMatcher;
    private Notifier m_loopNotifier;
    private boolean mLoopRunning;
    private boolean mLoopOverrun;

    private RGBY mCurrBuffColor, mCurrConfColor;
    private int mCurrBuffCount;

    private final Color kBlueTarget = new Color(0.11, 0.40, 0.48);    //0.143, 0.427, 0.429
    private final Color kGreenTarget = new Color(0.16, 0.59, 0.24);   //0.197, 0.561, 0.240
    private final Color kRedTarget = new Color(0.54, 0.32, 0.11);     //0.561, 0.232, 0.114
    private final Color kYellowTarget = new Color(0.32, 0.56, 0.11); //0.361, 0.524, 0.113

    public ColorSensor(I2C.Port port) {
        m_colorSensor = new ColorSensorV3(port);
        m_colorSensor.configureColorSensor(ColorSensorResolution.kColorSensorRes13bit, ColorSensorMeasurementRate.kColorRate25ms, GainFactor.kGain1x);
        m_colorMatcher = new ColorMatch();

        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kGreenTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kYellowTarget);
        m_colorMatcher.addColorMatch(kGreenTarget);

        m_loopNotifier = new Notifier(new CrashTrackingRunnable(){

            @Override
            public void runCrashTracked() {
                double t = Timer.getFPGATimestamp();
                updateColor();
                double dt_ms = (Timer.getFPGATimestamp() - t) * 1000.0;
                SmartDashboard.putNumber("dt_ms", dt_ms);
                if (dt_ms > 20.0) {
                    synchronized(ColorSensor.this) {
                        mLoopOverrun = true;
                    }
                }
            }
            
        });
        mLoopRunning = false;
    }

    public enum RGBY {
        RED, GREEN, BLUE, YELLOW, UNKNOWN
    }

    public synchronized void runLoop() {
        if (mLoopRunning) return;
        m_loopNotifier.startPeriodic(0.01); //kLooperDt
        mLoopRunning = true;
        mLoopOverrun = false;
        mCurrConfColor = RGBY.UNKNOWN;
    }

    public synchronized void stopLoop() {
        if (!mLoopRunning) return;
        m_loopNotifier.stop();
        mLoopRunning = false;
        mLoopOverrun = false;
        mCurrConfColor = RGBY.UNKNOWN;
    }

    private void updateColor() {
        RGBY c = getSensorColor();
        if (!getSensorProxDist()) c = RGBY.UNKNOWN;
        if (c != mCurrBuffColor) {
            mCurrBuffColor = c;
            mCurrBuffCount = 0;
        } else {
            mCurrBuffCount++;
        }

        if (mCurrBuffCount >= CONFIDENT_MINCOUNT) {
            // confirm change
            mCurrBuffCount = 0;
            synchronized(ColorSensor.this) {
                if (mCurrConfColor == RGBY.UNKNOWN
                    && mCurrBuffColor != RGBY.UNKNOWN) {
                    mCurrConfColor = mCurrBuffColor;
                } else if (mCurrBuffColor != RGBY.UNKNOWN
                        && mCurrBuffColor != mCurrConfColor) {

                }
            }
        }
    }

    private RGBY getSensorColor() {
        Color detectedColor = m_colorSensor.getColor();
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);      // matchClosest
        if (true) {
            SmartDashboard.putNumber("DET R", detectedColor.red);
            SmartDashboard.putNumber("DET G", detectedColor.green);
            SmartDashboard.putNumber("DET B", detectedColor.blue);
            if (match != null) {
                SmartDashboard.putNumber("COLOR CONF", match.confidence);
            }
        }
        if (match == null || match.confidence < CONF_BARRIER)
            return RGBY.UNKNOWN;
        else if (match.color == kBlueTarget) {
            return RGBY.BLUE;
        } else if (match.color == kGreenTarget) {
            return RGBY.GREEN;
        } else if (match.color == kRedTarget) {
            return RGBY.RED;
        } else if (match.color == kYellowTarget) {
            return RGBY.YELLOW;
        } else {
            return RGBY.UNKNOWN;
        }
    }

    // true if close enough
    private boolean getSensorProxDist() {
        if (true) {
            SmartDashboard.putNumber("COLOR PROX", m_colorSensor.getProximity());
        }
        return (m_colorSensor.getProximity() >= PROX_CLOSE_BOUND);
    }

    public synchronized RGBY getCurrColor() {
        return mCurrConfColor;
    }

    public synchronized boolean isLoopErr() {
        return mLoopOverrun;
    }

    

}
