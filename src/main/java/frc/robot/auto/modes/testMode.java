package frc.robot.auto.modes;

import java.util.List;

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

public class testMode extends AutoMode {

    private static final TrajectoryConfig trajConfig = new TrajectoryConfig(SwerveDriveModule.DRIVE_MAXV_MS * 0.5, SwerveDriveModule.DRIVE_MAXA_MS2 * 0.4);
    public testMode() {
        super(new GPose2d());
        
    }

    private PathFollowConfig getPath1(){
        PathFollowConfig pathConfig = new PathFollowConfig();

        pathConfig.trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(-90)),
                List.of(new Translation2d(0,-2.0)),
            new Pose2d(1.5, -2.0, Rotation2d.fromDegrees(0)),
            trajConfig
            );

        pathConfig.setInitPose = true;
        pathConfig.resetHeading = true;
        pathConfig.startHeading = 0.0;
        pathConfig.endHeading = 0.0;
        pathConfig.rotateDelay = 0.1;
        pathConfig.mRotateMethod = RotateMethod.PROFILE;

        return pathConfig;
    }
    
}
