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
import frc.robot.auto.actions.IntakeHomeAction;
import frc.robot.auto.actions.OuttakeAction;
import frc.robot.auto.actions.PathFollowAction;
import frc.robot.auto.actions.AutoShootAction;
import frc.robot.auto.actions.PathFollowAction.PathFollowConfig;
import frc.robot.auto.actions.PathFollowAction.RotateMethod;
import frc.robot.subsystems.SwerveDriveModule;

public class R4OMode extends AutoMode {

    public R4OMode() {
        super(new GPose2d());

        //get Ball 4
        addAction(new PathFollowAction(1.0, 2.5, getStartToBall4()));
        addAction(new IntakeAction(1.0, 1.1));
        addAction(new IntakeHomeAction(2.5, 2.6));


        //shoot Ball 4 and Preload
        addAction(new AutoShootAction(0.1, 5.0));

        //get Ball Terminal
        addAction(new PathFollowAction(5.0, 7.5, getBall4ToTerminal()));
        addAction(new IntakeAction(5.5, 5.6));
        addAction(new IntakeHomeAction(10.0, 10.1));


        //shoot Ball Terminal
        addAction(new PathFollowAction(10.0, 13.0, getTerminalToShoot()));
        addAction(new AutoShootAction(12.5, 15.0));



    }

    
    //1.29s
    private PathFollowConfig getStartToBall4() {
        PathFollowConfig pathConfig = new PathFollowConfig();

        pathConfig.trajectory = PathPlanner.loadPath("Right Start To Ball 4", new PathConstraints(4.0, 3));
        pathConfig.setInitPose = true;
        pathConfig.resetHeading = true;
        pathConfig.startHeading = 0.0;
        pathConfig.endHeading = 0.0;
        pathConfig.rotateDelay = 0.1;
        pathConfig.mRotateMethod = RotateMethod.PROFILE;

        return pathConfig;
    }
    
    //2.30s
    private PathFollowConfig getBall4ToTerminal() {
        PathFollowConfig pathConfig = new PathFollowConfig();

        pathConfig.trajectory = PathPlanner.loadPath("Ball 4 To Terminal", new PathConstraints(4.0, 3));
        pathConfig.setInitPose = false;
        pathConfig.resetHeading = false;
        pathConfig.startHeading = 0.0;
        pathConfig.endHeading = 0.0;
        pathConfig.rotateDelay = 0.1;
        pathConfig.mRotateMethod = RotateMethod.PROFILE;

        return pathConfig;
    }
    
    //2.85s
    private PathFollowConfig getTerminalToShoot() {
        PathFollowConfig pathConfig = new PathFollowConfig();

        pathConfig.trajectory = PathPlanner.loadPath("Terminal To Shoot", new PathConstraints(4.0, 3));
        pathConfig.setInitPose = false;
        pathConfig.resetHeading = false;
        pathConfig.startHeading = 0.0;
        pathConfig.endHeading = 0.0;
        pathConfig.rotateDelay = 0.1;
        pathConfig.mRotateMethod = RotateMethod.PROFILE;

        return pathConfig;
    }
    
}
