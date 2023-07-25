package frc.robot.subsystems;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib6907.loops.ILooper;
import frc.lib6907.loops.Loop;
import frc.lib6907.subsystem.Subsystem;
import frc.robot.subsystems.Intaker.IntakerState;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.SuperStructure.SuperStructureState;

public class LED extends Subsystem {

    AddressableLED mLED;
    AddressableLEDBuffer mLEDBuffer;

    private enum LEDState {
        Disabled,
        Idle,
        Intaking, 
        Shooting_Spinning, 
        Shooting
    }

    private static LED mInstance;
    private LEDState mLEDState = LEDState.Disabled;

    public synchronized static LED getInstance() {
        if (mInstance == null) {
            mInstance = new LED();
        }

        return mInstance;
    }

    private int mRainbowFirstPixelHue = 0;
    private final int LED_LENGTH = 45;

    private LED() {
        mLED = new AddressableLED(1);
        mLEDBuffer = new AddressableLEDBuffer(LED_LENGTH);
        mLED.setLength(LED_LENGTH);
        mLED.setData(mLEDBuffer);
        mLED.start();
    }

    @Override
    public void stop() {

    }
    
    @Override
    public synchronized void readPeriodicInputs() {

        determineState();
        SmartDashboard.putString("LED: State", mLEDState.toString());
        switch (mLEDState) {
            case Disabled:
                setRisingColor(Color.kDarkOrange);
                break;
            case Idle:
            default:
                if (DriverStation.getAlliance() == Alliance.Red)
                    setRisingColor(Color.kRed);
                else if (DriverStation.getAlliance() == Alliance.Blue)
                    setRisingColor(Color.kBlue);
                break;
            case Intaking:
                setRisingColor(Color.kPurple);
                break;
            case Shooting:
                setRainbow();
                break;
            case Shooting_Spinning:
                setBlinkingColor(Color.kYellowGreen);
                break;
        }
        
        mLED.setData(mLEDBuffer);
    }
    
    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
                synchronized (LED.this) {

                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (LED.this) {

                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }

        });
    }
    
    private synchronized void determineState() {
        IntakerState mIS = Intaker.getInstance().getState();
        SuperStructureState mSS = SuperStructure.getInstance().getState();
        boolean disabled = DriverStation.isDisabled();

        if (disabled) {
            mLEDState = LEDState.Disabled;
        }else if(mSS == SuperStructureState.AIMING || mSS == SuperStructureState.SHOOTING){
            mLEDState = LEDState.Shooting;
        } else if (mIS == IntakerState.INTAKE){
            mLEDState = LEDState.Intaking;
        }else{ 
            mLEDState = LEDState.Idle;
        } 

        
    }

    private synchronized void setRainbow() {
        int temp = Math.min(LED_LENGTH-1,(int)(LED_LENGTH * Shooter.getInstance().getSpinningPercentage()));
        if(temp >= 42){
            mRainbowFirstPixelHue = (mRainbowFirstPixelHue + 1) % 180;
            for (int i = 0; i < LED_LENGTH; i++) {
                final int hue = (mRainbowFirstPixelHue + (i * 180 / LED_LENGTH)) % 180;
                mLEDBuffer.setHSV(i, hue, 255, 128);
            }
            mRainbowFirstPixelHue += 3;
            mRainbowFirstPixelHue %= 180;
        } else {
            try{
                for (int i = 0; i < temp; i++) {
                    final int hue = (i * 180 / LED_LENGTH) % 180;
                    mLEDBuffer.setHSV(i, hue, 255, 128);
                }
                for (int i = temp; i < LED_LENGTH; i++) {
                    mLEDBuffer.setRGB(i, 0, 0, 0);
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }


    private void setThreeColor() {
        Color top = Color.kDarkMagenta;
        Color mid = Color.kDarkBlue;
        Color low = Color.kDarkCyan;

        if (true) {
            top = Color.kMagenta;
        }
        if (true) {
            mid = Color.kBlue;
        }
        if (true) {
            low = Color.kCyan;
        }

        int length = LED_LENGTH;
        for (int i = 0; i < length; i++) {
            if (i < length / 3) {
                mLEDBuffer.setLED(i, low);
            } else if (i < length / 3 * 2) {
                mLEDBuffer.setLED(i, mid);
            } else {
                mLEDBuffer.setLED(i, top);
            }
        }
    }

    private int brightPosition = 0;
    private final double TAIL_LENGTH = 30.0;

    public synchronized void setRisingColor(Color color) {
        Color8Bit color8bit = new Color8Bit(color);
        brightPosition += 1;
        if (brightPosition >= LED_LENGTH)
            brightPosition = 0;
        for (int i = 0; i < LED_LENGTH; i++) {
            mLEDBuffer.setRGB(i, 0, 0, 0);
        }
        for (int i = 0; i < TAIL_LENGTH; i++) {
            mLEDBuffer.setRGB((brightPosition + LED_LENGTH + i)%LED_LENGTH, (int) (color8bit.red / TAIL_LENGTH * i),
                    (int) (color8bit.green / TAIL_LENGTH * i), (int) (color8bit.blue / TAIL_LENGTH * i));
        }
    }
    
    public void setBlinkingColor(Color color) {
        if (Timer.getFPGATimestamp() % 1 > 0.5) {
            for (int i = 0; i < LED_LENGTH; i++) {
                mLEDBuffer.setLED(i, color);
            }

        } else {
            for (int i = 0; i < LED_LENGTH; i++) {
                mLEDBuffer.setLED(i, Color.kBlack);
            }
        }
    }

    public void setSolidColor(Color color) {
        for (int i = 0; i < LED_LENGTH; i++){
            mLEDBuffer.setLED(i, color);
        }
    }

    @Override
    public void writePeriodicOutputs() {
        
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
    }
    
    
}
