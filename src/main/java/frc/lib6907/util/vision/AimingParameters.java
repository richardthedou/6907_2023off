package frc.lib6907.util.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib6907.geometry.GPose2d;
import frc.lib6907.geometry.GRotation2d;

public class AimingParameters {
    private final double range;
    private final GPose2d field_to_vehicle;
    private final GPose2d field_to_goal;
    private final GRotation2d robot_to_goal_rotation;
    private final double last_seen_timestamp;
    private final double stability;
    private final int track_id;

    public AimingParameters(Pose2d field_to_vehicle,
                            Pose2d field_to_goal, double last_seen_timestamp,
            double stability, int track_id) {
        this.field_to_vehicle = new GPose2d(field_to_vehicle);
        this.field_to_goal = new GPose2d(field_to_goal);
        final GPose2d vehicle_to_goal = this.field_to_vehicle.inverse().transformBy(field_to_goal);
        this.range = vehicle_to_goal.getTranslation().getNorm();
        this.robot_to_goal_rotation = vehicle_to_goal.getTranslation().direction();
        this.last_seen_timestamp = last_seen_timestamp;
        this.stability = stability;
        this.track_id = track_id;
    }
    
    public AimingParameters(Pose2d field_to_vehicle, Pose2d field_to_goal, double last_seen_timestamp) {
        this(field_to_vehicle, field_to_goal, last_seen_timestamp, 0.0, 0);
    }

    public GPose2d getFieldToVehicle() {
        return field_to_vehicle;
    }

    public GPose2d getFieldToGoal() {
        return field_to_goal;
    }

    public double getRange() {
        return range;
    }

    public GRotation2d getRobotToGoalRotation() {
        return robot_to_goal_rotation;
    }

    public double getLastSeenTimestamp() {
        return last_seen_timestamp;
    }

    public double getStability() {
        return stability;
    }

    public int getTrackId() {
        return track_id;
    }
}