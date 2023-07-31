package frc.robot.auto.modes;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

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
import frc.robot.auto.actions.ShootManualAction;
import frc.robot.auto.actions.AutoShootAction;
import frc.robot.auto.actions.PathFollowAction.PathFollowConfig;
import frc.robot.auto.actions.PathFollowAction.RotateMethod;
import frc.robot.subsystems.SwerveDriveModule;
import frc.robot.util.ShootingParameters;

public class R5OMode extends AutoMode {

    List<PathPlannerTrajectory> paths;
       

    public R5OMode() {
        super(new GPose2d());

        paths = PathPlanner.loadPathGroup("Right 4 Ball", List.of(
            new PathConstraints(4, 3),
            new PathConstraints(4, 3),
            new PathConstraints(4, 3)
        ));

        //get Ball 4
        addAction(new PathFollowAction(1.0, 2.5, getStartToBall4()));
        addAction(new IntakeAction(1.0, 1.1));
        addAction(new IntakeHomeAction(2.5, 2.6));


        //shoot Ball 4 and Preload
        addAction(new AutoShootAction(0.1, 5.0));

        //get Ball Terminal
        addAction(new PathFollowAction(5.0, 7.5, getBall4ToTerminal()));
        addAction(new IntakeAction(5.5, 5.6));
        addAction(new IntakeHomeAction(9.0, 9.1));


        //shoot Ball Terminal
        addAction(new PathFollowAction(9.0, 12.0, getTerminalToShoot()));
        addAction(new AutoShootAction(11.0, 15.0));
        // addAction(new ShootManualAction(11.0, 15.0, new ShootingParameters(8850, 13500, 75)));

        addAction(new PathFollowAction(12.0, 14.0, getShootToBall5()));
        addAction(new IntakeAction(12.0, 12.1));
        addAction(new IntakeHomeAction(13.7, 13.8));


    }

    
    //1.29s
    private PathFollowConfig getStartToBall4() {
        PathFollowConfig pathConfig = new PathFollowConfig();
        pathConfig.trajectory = paths.get(0);
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

        pathConfig.trajectory = paths.get(1);
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

        pathConfig.trajectory = paths.get(2);
        pathConfig.setInitPose = false;
        pathConfig.resetHeading = false;
        pathConfig.startHeading = 0.0;
        pathConfig.endHeading = 0.0;
        pathConfig.rotateDelay = 0.1;
        pathConfig.mRotateMethod = RotateMethod.PROFILE;

        return pathConfig;
    }
    
    private PathFollowConfig getShootToBall5() {
        PathFollowConfig pathConfig = new PathFollowConfig();

        pathConfig.trajectory = PathPlanner.loadPath("Right Last Ball", new PathConstraints(0.3, 3.0));
        pathConfig.setInitPose = false;
        pathConfig.resetHeading = false;
        pathConfig.startHeading = 0.0;
        pathConfig.endHeading = 0.0;
        pathConfig.rotateDelay = 0.1;
        pathConfig.mRotateMethod = RotateMethod.PROFILE;

        return pathConfig;
    }
}
