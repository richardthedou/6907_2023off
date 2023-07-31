package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib6907.auto.AutoMode;
import frc.robot.auto.modes.L2O2XMode;
import frc.robot.auto.modes.PathPlannerTestMode;
import frc.robot.auto.modes.R4OMode;
import frc.robot.auto.modes.R4ONewMode;
import frc.robot.auto.modes.R5OMode;
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
        mChooser.addOption("Left 2O 2X Mode", new L2O2XMode());
        mChooser.addOption("Right 4O Mode", new R4OMode());
        mChooser.addOption("Right 4O New Mode", new R4ONewMode());
        mChooser.addOption("Right 5O Mode", new R5OMode());
        mChooser.addOption("Path Planner Test Mode", new PathPlannerTestMode());



        SmartDashboard.putData("AUTO CHOICES", mChooser);
    }

    public AutoMode getSelected() {
        return mChooser.getSelected();
    }
    
}
