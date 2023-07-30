package frc.robot.auto.modes;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;


import frc.lib6907.auto.AutoMode;
import frc.lib6907.geometry.GPose2d;
import frc.robot.auto.actions.IntakeAction;
import frc.robot.auto.actions.PathFollowAction;
import frc.robot.auto.actions.AutoShootAction;
import frc.robot.auto.actions.PathFollowAction.PathFollowConfig;
import frc.robot.auto.actions.PathFollowAction.RotateMethod;

public class R5OMode extends AutoMode {

    public R5OMode() {
        super(new GPose2d());

        addAction(new AutoShootAction(0.1, 15.0));

        //get Ball 5 and Shoot Preload
        addAction(new PathFollowAction(1.0, 2.2, getStartToBall5()));
        addAction(new IntakeAction(1.0, 2.7));

        //get Ball 4 and Shoot
        addAction(new PathFollowAction(3.7, 5.2, getBall5ToBall4()));
        addAction(new IntakeAction(4.0, 6.0));


        //get Ball Terminal
        addAction(new PathFollowAction(7.05, 9.5, getBall4ToTerminal2()));
        addAction(new IntakeAction(8.5, 11.2));

        //shoot Ball Terminal
        addAction(new PathFollowAction(11.2, 13.5, getTerminalToShoot2()));

    }

    
    //1.12s
    private PathFollowConfig getStartToBall5() {
        PathFollowConfig pathConfig = new PathFollowConfig();

        pathConfig.trajectory = PathPlanner.loadPath("Start To Ball 5", new PathConstraints(4.0, 3));
        pathConfig.setInitPose = true;
        pathConfig.resetHeading = true;
        pathConfig.startHeading = 0.0;
        pathConfig.endHeading = 0.0;
        pathConfig.rotateDelay = 0.1;
        pathConfig.mRotateMethod = RotateMethod.PROFILE;

        return pathConfig;
    }
    
    //1.82s
    private PathFollowConfig getBall5ToBall4() {
        PathFollowConfig pathConfig = new PathFollowConfig();

        pathConfig.trajectory = PathPlanner.loadPath("Ball 5 To Ball 4", new PathConstraints(4.0, 3));
        pathConfig.setInitPose = false;
        pathConfig.resetHeading = false;
        pathConfig.startHeading = 0.0;
        pathConfig.endHeading = 0.0;
        pathConfig.rotateDelay = 0.1;
        pathConfig.mRotateMethod = RotateMethod.PROFILE;

        return pathConfig;
    }
    
    //2.36s
    private PathFollowConfig getBall4ToTerminal2() {
        PathFollowConfig pathConfig = new PathFollowConfig();

        pathConfig.trajectory = PathPlanner.loadPath("Ball 4 To Terminal 2", new PathConstraints(4.0, 3));
        pathConfig.setInitPose = false;
        pathConfig.resetHeading = false;
        pathConfig.startHeading = 0.0;
        pathConfig.endHeading = 0.0;
        pathConfig.rotateDelay = 0.1;
        pathConfig.mRotateMethod = RotateMethod.PROFILE;

        return pathConfig;
    }

    //2.15
    private PathFollowConfig getTerminalToShoot2(){
        PathFollowConfig pathConfig = new PathFollowConfig();

        pathConfig.trajectory = PathPlanner.loadPath("Terminal To Shoot 2", new PathConstraints(4.0, 3));
        pathConfig.setInitPose = false;
        pathConfig.resetHeading = false;
        pathConfig.startHeading = 0.0;
        pathConfig.endHeading = 0.0;
        pathConfig.rotateDelay = 0.1;
        pathConfig.mRotateMethod = RotateMethod.PROFILE;

        return pathConfig;
    }
    
}
