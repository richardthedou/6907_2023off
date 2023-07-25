package frc.lib6907.control;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;

// NOT WORKING NOW

public class HolonomicPurePursuitTrajectoryFollower extends HolonomicTrajectoryFollower {

    private static final double kStartSearchTime = 0.05;
    private static final double kSearchDt = 0.005;
    private static final double kMinDist = 0.10;
    private static final double kStartV = 0.7; //max vel * 0.2s

    private HolonomicFeedforward velFF;
    private boolean hasReachStartV;

    public HolonomicPurePursuitTrajectoryFollower(HolonomicFeedforward ff) {
        velFF = ff;
        hasReachStartV = false;
    }

    @Override
    public double[] calculateDrivePercOutput(Pose2d currPose, double time, double dt) {
        Trajectory.State closestState = getClosestState(currPose, time);
        Trajectory.State lookaheadState = getLookAheadState(closestState, time);

        if (!hasReachStartV && 
            (closestState.velocityMetersPerSecond > kStartV /* || time > mTraj.getTotalTimeSeconds() / 2.0 */ ))
            hasReachStartV = true;
        double velMs = hasReachStartV ? closestState.velocityMetersPerSecond : kStartV;
        Translation2d vecDir = lookaheadState.poseMeters.relativeTo(currPose).getTranslation();
        Translation2d velVec = vecDir.times(velMs / vecDir.getNorm());
        Translation2d accelVec = vecDir.times(closestState.accelerationMetersPerSecondSq / vecDir.getNorm());

        Translation2d transVector = velFF.calculateFeedforward(velVec, accelVec);
        return new double[] { transVector.getX(), transVector.getY(),0};
    }

    private Trajectory.State getClosestState(Pose2d currPose, double time) {
        double searchStepSize = 0.01;
        double previewQuantity = 0.0;
    	double searchDirection = 1.0;
    	double forwardDistance = distance(currPose, previewQuantity + searchStepSize, time);
    	double reverseDistance = distance(currPose, previewQuantity - searchStepSize, time);
    	searchDirection = Math.signum(reverseDistance - forwardDistance);
        while(searchStepSize > 0.0001){
        	if(epsilonEquals(distance(currPose, previewQuantity, time), 0.0, 0.001)) break;
        	while(distance(currPose, previewQuantity + searchStepSize*searchDirection, time) < 
        			distance(currPose, previewQuantity, time)) {
        		previewQuantity += searchStepSize*searchDirection;
        	}
        	searchStepSize /= 10.0;
        	searchDirection *= -1;
        }

        return mTraj.sample(time + previewQuantity);
    }

    private Trajectory.State getLookAheadState(Trajectory.State closeState, double time) {
        double destTime = kStartSearchTime;
        while(distance(closeState.poseMeters, destTime, time) < kMinDist
            && time + destTime < mTraj.getTotalTimeSeconds()) {
                destTime += kSearchDt;
        }
        
        return mTraj.sample(time + destTime);
    }

    private double distance(Pose2d currPose, double prevTime, double time) {
        return mTraj.sample(time + prevTime).poseMeters.relativeTo(currPose).getTranslation().getNorm();
    }

    /*
    private double direction(Pose2d currPose, double time, double prev_q, double step_size) {
    	double forwardDistance = distance(currPose, prev_q + step_size, time);
    	double reverseDistance = distance(currPose, prev_q - step_size, time);
    	return Math.signum(reverseDistance - forwardDistance);
    }
    */

    private boolean epsilonEquals(double a, double b, double epsilon) {
        return Math.abs(a - b) < Math.abs(epsilon);
    }

    @Override
    public void reset() {
        super.reset();
        hasReachStartV = false;
    }
    
}
