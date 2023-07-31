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
import frc.robot.auto.actions.IntakeAction;
import frc.robot.auto.actions.IntakeHoldAction;
import frc.robot.auto.actions.IntakeHomeAction;
import frc.robot.auto.actions.OuttakeAction;
import frc.robot.auto.actions.PathFollowAction;
import frc.robot.auto.actions.ShootManualAction;
import frc.robot.auto.actions.AutoShootAction;
import frc.robot.auto.actions.PathFollowAction.PathFollowConfig;
import frc.robot.auto.actions.PathFollowAction.RotateMethod;
import frc.robot.subsystems.SwerveDriveModule;
import frc.robot.util.ShootingParameters;

public class L2O1XMode extends AutoMode {

    public L2O1XMode() {
        super(new GPose2d());

        //get Ball 2
        addAction(new PathFollowAction(1.0, 3.0, getStartToBall2()));
        addAction(new IntakeAction(1.0, 1.1));
        addAction(new IntakeHomeAction(3.0, 3.1));


        //shoot Ball 2 and Preload
        addAction(new AutoShootAction(0.1, 5.0));
        // addAction(new ShootManualAction(0.1, 5.0, new ShootingParameters(12300, 9000, -33)));

        //get Ball 1
        addAction(new PathFollowAction(4.5, 7.0, getBall2ToBall1()));
        addAction(new IntakeAction(5.5, 5.6));
        addAction(new IntakeHoldAction(7.4, 7.5));


        //to Hide
        addAction(new PathFollowAction(8.0, 10.2, getBall1ToHide2()));
        addAction(new OuttakeAction(9.0, 9.1));
        addAction(new IntakeHomeAction(13.0, 13.1));

        addAction(new PathFollowAction(13.0, 15.0, getHideToMid()));

    }

    
    //1.48s
    private PathFollowConfig getStartToBall2() {
        PathFollowConfig pathConfig = new PathFollowConfig();

        pathConfig.trajectory = PathPlanner.loadPath("Left Start To Ball 2", new PathConstraints(4.0, 2));
        pathConfig.setInitPose = true;
        pathConfig.resetHeading = true;
        pathConfig.startHeading = 0.0;
        pathConfig.endHeading = 0.0;
        pathConfig.rotateDelay = 0.1;
        pathConfig.mRotateMethod = RotateMethod.PROFILE;

        return pathConfig;
    }
    
    //2.39s
    private PathFollowConfig getBall2ToBall1() {
        PathFollowConfig pathConfig = new PathFollowConfig();

        pathConfig.trajectory = PathPlanner.loadPath("Ball 2 To Ball 1", new PathConstraints(4.0, 2));
        pathConfig.setInitPose = false;
        pathConfig.resetHeading = false;
        pathConfig.startHeading = 0.0;
        pathConfig.endHeading = 0.0;
        pathConfig.rotateDelay = 0.1;
        pathConfig.mRotateMethod = RotateMethod.PROFILE;

        return pathConfig;
    }
    
    //2.14s
    private PathFollowConfig getBall1ToHide2() {
        PathFollowConfig pathConfig = new PathFollowConfig();

        pathConfig.trajectory = PathPlanner.loadPath("Ball 1 To Hide 2", new PathConstraints(4.0, 2));
        pathConfig.setInitPose = false;
        pathConfig.resetHeading = false;
        pathConfig.startHeading = 0.0;
        pathConfig.endHeading = 0.0;
        pathConfig.rotateDelay = 0.1;
        pathConfig.mRotateMethod = RotateMethod.PROFILE;

        return pathConfig;
    }

    //1.95s
    private PathFollowConfig getHideToMid(){
        PathFollowConfig pathConfig = new PathFollowConfig();

        pathConfig.trajectory = PathPlanner.loadPath("Hide To Mid", new PathConstraints(4.0, 2));
        pathConfig.setInitPose = false;
        pathConfig.resetHeading = false;
        pathConfig.startHeading = 0.0;
        pathConfig.endHeading = 0.0;
        pathConfig.rotateDelay = 0.1;
        pathConfig.mRotateMethod = RotateMethod.PROFILE;

        return pathConfig;
    }
    
}
