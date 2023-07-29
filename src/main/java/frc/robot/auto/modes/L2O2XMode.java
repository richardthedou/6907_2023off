package frc.robot.auto.modes;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.lib6907.auto.AutoMode;
import frc.lib6907.geometry.GPose2d;
import frc.robot.auto.actions.PathFollowAction;
import frc.robot.auto.actions.PathFollowAction.PathFollowConfig;
import frc.robot.auto.actions.PathFollowAction.RotateMethod;
import frc.robot.subsystems.SwerveDriveModule;

public class L2O2XMode extends AutoMode {

    public L2O2XMode() {
        super(new GPose2d());
        
    }

    private PathFollowConfig getStartToBall2() {
        PathFollowConfig pathConfig = new PathFollowConfig();

        pathConfig.trajectory = PathPlanner.loadPath("Start To Ball 2", new PathConstraints(4.0, 3));
        pathConfig.setInitPose = true;
        pathConfig.resetHeading = true;
        pathConfig.startHeading = 0.0;
        pathConfig.endHeading = 0.0;
        pathConfig.rotateDelay = 0.1;
        pathConfig.mRotateMethod = RotateMethod.PROFILE;

        return pathConfig;
    }
    
    private PathFollowConfig getBall2ToBall1() {
        PathFollowConfig pathConfig = new PathFollowConfig();

        pathConfig.trajectory = PathPlanner.loadPath("Ball 2 To Ball 1", new PathConstraints(4.0, 3));
        pathConfig.setInitPose = false;
        pathConfig.resetHeading = false;
        pathConfig.startHeading = 0.0;
        pathConfig.endHeading = 0.0;
        pathConfig.rotateDelay = 0.1;
        pathConfig.mRotateMethod = RotateMethod.PROFILE;

        return pathConfig;
    }
    
    private PathFollowConfig getBall1ToBall3() {
        PathFollowConfig pathConfig = new PathFollowConfig();

        pathConfig.trajectory = PathPlanner.loadPath("Ball 1 To Ball 3", new PathConstraints(4.0, 3));
        pathConfig.setInitPose = false;
        pathConfig.resetHeading = false;
        pathConfig.startHeading = 0.0;
        pathConfig.endHeading = 0.0;
        pathConfig.rotateDelay = 0.1;
        pathConfig.mRotateMethod = RotateMethod.PROFILE;

        return pathConfig;
    }
    private PathFollowConfig getBall3ToHide(){
        PathFollowConfig pathConfig = new PathFollowConfig();

        pathConfig.trajectory = PathPlanner.loadPath("1 Cone Charge", new PathConstraints(4.0, 3));
        pathConfig.setInitPose = true;
        pathConfig.resetHeading = true;
        pathConfig.startHeading = 0.0;
        pathConfig.endHeading = 0.0;
        pathConfig.rotateDelay = 0.1;
        pathConfig.mRotateMethod = RotateMethod.PROFILE;

        return pathConfig;
    }
    
}
