package frc.robot.auto.modes;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.lib6907.auto.AutoMode;
import frc.lib6907.geometry.GPose2d;
import frc.robot.auto.actions.PathFollowAction;
import frc.robot.auto.actions.PathFollowAction.PathFollowConfig;
import frc.robot.auto.actions.PathFollowAction.RotateMethod;

public class PathPlannerTestMode extends AutoMode {
    PathPlannerTrajectory testPath;

    
    public PathPlannerTestMode() {
        super(new GPose2d());
        addAction(new PathFollowAction(0.5, 10.0, getPath1()));
    }

    private PathFollowConfig getPath1(){
        PathFollowConfig pathConfig = new PathFollowConfig();
        testPath = PathPlanner.loadPath("test path 2", new PathConstraints(3, 3));
        pathConfig.trajectory = testPath;

        pathConfig.setInitPose = true;
        pathConfig.resetHeading = true;
        pathConfig.startHeading = 0.0;
        pathConfig.endHeading = 0.0;
        pathConfig.rotateDelay = 0.1;
        pathConfig.mRotateMethod = RotateMethod.PROFILE;

        return pathConfig;
    }
    
}
