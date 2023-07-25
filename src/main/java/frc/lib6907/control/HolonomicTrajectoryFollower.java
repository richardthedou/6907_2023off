package frc.lib6907.control;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;

public abstract class HolonomicTrajectoryFollower {
    
    protected Trajectory mTraj = new Trajectory();

    public void setTrajectory(Trajectory traj) {
        mTraj = traj;
    }

    /**
     * 
     * @param currPose
     * @param time
     * @param dt
     * @return {x,y,holonomic rotation vel}
     */
    public abstract double[] calculateDrivePercOutput(Pose2d currPose, double time, double dt);



    public void reset() {
        mTraj = new Trajectory();
    }

}
