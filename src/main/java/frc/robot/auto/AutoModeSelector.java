package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib6907.auto.AutoMode;
import frc.robot.auto.modes.testMode;

public class AutoModeSelector {

    private SendableChooser<AutoMode> mChooser;

    private static AutoModeSelector sInstance;
    public static AutoModeSelector getInstance() {
        if (sInstance == null) sInstance = new AutoModeSelector();
        return sInstance;
    }
    private AutoModeSelector() {
        // init points
        mChooser = new SendableChooser<>();


       
        mChooser.addOption("Test Mode", new testMode());

        SmartDashboard.putData("AUTO CHOICES", mChooser);
    }

    public AutoMode getSelected() {
        return mChooser.getSelected();
    }
    
}
