package frc.robot.auto.actions;

import java.util.ArrayList;
import java.util.TreeMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.lib6907.auto.Action;
import frc.robot.subsystems.SwerveDrive;

public class PathFollowAction extends Action {

    private double timesft;
    private PathFollowConfig mConfig;

    public enum RotateMethod {
        PROFILE, LIMELIGHT, AUTO_LOCK_TARGET, PIXY
    }

    public static class PathFollowConfig {
        public Trajectory trajectory = new Trajectory();
        public boolean resetHeading = false;
        public boolean setInitPose = false;
        public double startHeading = 0.0;
        public double endHeading = 0.0;
        public double rotateDelay = 0.0;
        public ArrayList<double[]> rotateCriticalPoints = new ArrayList<double[]>(); // double[] = {time, heading}
        public TreeMap<Double, Double> adjustedCPs = new TreeMap<Double, Double>();
        public RotateMethod mRotateMethod;

    }

    public PathFollowAction(double start, double end, PathFollowConfig config) {
        super(start, end);
        mConfig = config;

        switch (config.mRotateMethod) {
            case AUTO_LOCK_TARGET:
                break;
            case LIMELIGHT:
                break;
            default:
            case PROFILE:
                break;
            case PIXY:
                break;
        }
    }

    @Override
    public void start() {
        SwerveDrive.getInstance().setFollowTrajectory(mConfig);
        SwerveDrive.getInstance().m_field.getObject("traj").setTrajectory(mConfig.trajectory);
        timesft = Timer.getFPGATimestamp();
    }

    @Override
    public void run() {
        double timestamp = Timer.getFPGATimestamp() - timesft;

        switch (mConfig.mRotateMethod) {
            case LIMELIGHT:
                break;
            default:
            case PROFILE:
                PathPlannerState targetState = ((PathPlannerState) ((PathPlannerTrajectory) mConfig.trajectory)
                .sample(timestamp));
                SwerveDrive.getInstance().putAutoTargetState(targetState); //on field 2d
                SwerveDrive.getInstance().rotate(targetState.holonomicRotation.getDegrees(), true, targetState.holonomicAngularVelocityRadPerSec*180/Math.PI);
                break;
            case PIXY:
                break;
        }
    }

    @Override
    public void end() {
        SwerveDrive.getInstance().stop();
    }

}
