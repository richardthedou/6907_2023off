package frc.lib6907.util.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import frc.lib6907.geometry.GPose2d;

/**
 * A container class for Targets detected by the vision system, containing the location in three-dimensional space.
 */
public class TargetInfo {
    // protected double x,y,z;
    protected Rotation2d r2d;
    protected Translation2d t2d;
    protected int id;

    public TargetInfo(double x, double y, Rotation2d rotation, int id) {
        this.t2d = new Translation2d(x, y);
        this.r2d = rotation;
        this.id = id;
    }

    public TargetInfo(Pose2d pose, int id){
        this(pose.getX(), pose.getY(), pose.getRotation(), id);
    }

    public Translation2d geTranslation3d(){
        return t2d;
    }

    public Rotation2d getRotation3d(){
        return r2d;
    }

    public Pose2d getPose2d(){
        return new GPose2d(t2d, r2d);
    }

    public int getID(){
        return this.id;
    }

    public TargetInfo TransformedPose(Pose2d cameraPoseToRobot, Pose2d RobotPoseToField){
         Pose2d new_pose = new GPose2d(t2d, r2d).transformBy(new GPose2d(cameraPoseToRobot)).transformBy(new GPose2d(RobotPoseToField));
         this.t2d = new_pose.getTranslation();
         this.r2d = new_pose.getRotation();
         return this;
    }

    public String toString(){
        return "ID:"+this.id + " Pose" + new GPose2d(this.getPose2d()).toString();
    }
}