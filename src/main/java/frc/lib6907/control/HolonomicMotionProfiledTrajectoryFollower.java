package frc.lib6907.control;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib6907.util.SynchronousPIDF;

public class HolonomicMotionProfiledTrajectoryFollower extends HolonomicTrajectoryFollower {
    private SynchronousPIDF forwardController;
    private SynchronousPIDF strafeController;

    private HolonomicFeedforward feedforward;

    public HolonomicMotionProfiledTrajectoryFollower(SynchronousPIDF translationPIDF, HolonomicFeedforward feedforward) {
        this.forwardController = new SynchronousPIDF(translationPIDF.getP(), translationPIDF.getI(), translationPIDF.getD(), translationPIDF.getF());
        this.strafeController = new SynchronousPIDF(translationPIDF.getP(), translationPIDF.getI(), translationPIDF.getD(), translationPIDF.getF());

        this.feedforward = feedforward;
    }

    @Override
    public double[] calculateDrivePercOutput(Pose2d currPose, double time, double dt) {
        PathPlannerTrajectory.PathPlannerState state = (PathPlannerState) mTraj.sample(time);

        Translation2d segmentVelocity = new Translation2d(state.velocityMetersPerSecond, state.poseMeters.getRotation());
        Translation2d segmentAcceleration = new Translation2d(state.accelerationMetersPerSecondSq, state.poseMeters.getRotation());

        Translation2d feedforwardVector = feedforward.calculateFeedforward(segmentVelocity, segmentAcceleration);

        
        Pose2d target = new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation);
        SmartDashboard.putString("SWERVE: AUTO TARGET", state.poseMeters.getTranslation().toString() + " Rotation:" + state.holonomicRotation.getDegrees());
        PathPlannerServer.sendPathFollowingData(target, currPose);
            
        forwardController.setSetpoint(state.poseMeters.getX());
        strafeController.setSetpoint(state.poseMeters.getY());


        double[] control_output = new double[]{forwardController.calculate(currPose.getTranslation().getX(), dt) + feedforwardVector.getX(),strafeController.calculate(currPose.getTranslation().getY(), dt) + feedforwardVector.getY(), state.holonomicRotation.getDegrees()};
        return control_output;
    }

    @Override
    public void reset() {
        super.reset();
        forwardController.reset();
        strafeController.reset();
    }

    
}
