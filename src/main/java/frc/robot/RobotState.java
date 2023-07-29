package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Map.Entry;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib6907.geometry.GPose2d;
import frc.lib6907.geometry.GRotation2d;
import frc.lib6907.geometry.GTranslation2d;
import frc.lib6907.math.InterpolatingDouble;
import frc.lib6907.math.InterpolatingTreeMap;
import frc.lib6907.util.vision.AimingParameters;
import frc.lib6907.util.vision.GoalTracker;
import frc.lib6907.util.vision.TargetInfo;
import frc.lib6907.util.vision.GoalTracker.TrackReport;
import frc.robot.subsystems.SwerveDrive;

public class RobotState {
    private static RobotState mInstance;

    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }
        return mInstance;
    }

    private static final int kPose_Buffer_Size = 100;

    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> vehicle_to_turret_;
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle;
    private SwerveDrivePoseEstimator field_to_robot_est;
    private SwerveDriveOdometry field_to_robot_odom;

    private GPose2d field_to_vt = new GPose2d();
    private final GPose2d theoretical_field_to_vt = new GPose2d();
    private GTranslation2d vehicle_to_turret_translation = new GTranslation2d(0.15, 0);
    private Pose2d cameraToRobot = new Pose2d(0,0, new Rotation2d());
    private GoalTracker goal_tracker = new GoalTracker();

    
    private RobotState() {
        reset();
    }

    public synchronized void reset(double start_time, GPose2d initial_field_to_vehicle,
            GPose2d initial_vehicle_to_turret) {
        reset(start_time, initial_field_to_vehicle);
        vehicle_to_turret_ = new InterpolatingTreeMap<>(kPose_Buffer_Size);
        vehicle_to_turret_.put(new InterpolatingDouble(start_time), initial_vehicle_to_turret);
    }

    public synchronized void reset(double start_time, GPose2d initial_field_to_vehicle) {
        field_to_robot_est = new SwerveDrivePoseEstimator(initial_field_to_vehicle.getRotation(),
                initial_field_to_vehicle, SwerveDrive.getInstance().getKinematics(),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01),
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.02),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01));
        field_to_vehicle = new InterpolatingTreeMap<>(kPose_Buffer_Size);
        field_to_vehicle.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
    }

    public synchronized void reset() {
        reset(Timer.getFPGATimestamp(), new GPose2d(), new GPose2d(vehicle_to_turret_translation, new GRotation2d()));
    }

    public synchronized GPose2d getFieldToVehicle(double timestamp) {
        return new GPose2d(field_to_vehicle.getInterpolated(new InterpolatingDouble(timestamp)));
    }

    public synchronized GPose2d getOdom() {
        return new GPose2d(field_to_robot_odom.getPoseMeters());
    }

    public synchronized Pose2d getPoseEstimate(){
        return field_to_robot_est.getEstimatedPosition();
    }


    public synchronized GPose2d getVehicleToTurret(double timestamp) {
        return new GPose2d(vehicle_to_turret_.getInterpolated(new InterpolatingDouble(timestamp)));
    }

    public synchronized GPose2d getFieldToTurret(double timestamp) {
        return getFieldToVehicle(timestamp).transformBy(getVehicleToTurret(timestamp));
    }

    public synchronized Entry<InterpolatingDouble, Pose2d> getLatestVehicleToTurret() {
        return vehicle_to_turret_.lastEntry();
    }

    public synchronized GPose2d getFieldToVehicleFromVision() {
        return new GPose2d(getVehicleToVT().inverse().rotateBy(SwerveDrive.getInstance().getHeading()), 
            SwerveDrive.getInstance().getHeading());
    }

    public synchronized void updateFieldToVehicleFromOdom(double timestamp, SwerveModuleState[] states) {
        field_to_robot_est.updateWithTime(timestamp, SwerveDrive.getInstance().getHeading(), states);
        Pose2d curr = field_to_robot_est.getEstimatedPosition();
        field_to_vehicle.put(new InterpolatingDouble(timestamp), curr);
    }

    public synchronized void updateFieldtoVehicleFromVision(double timestamp) {
        field_to_robot_est.addVisionMeasurement(getFieldToVehicleFromVision(), timestamp,
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01));
    }
    
    public synchronized void updateVehicleToTurret(double timestamp, GPose2d observation) {
        vehicle_to_turret_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void resetVision() {
        // lastVisionUpdate = Double.NEGATIVE_INFINITY;
        // field_to_vt = theoretical_field_to_vt;
        goal_tracker.reset();
    }

    private double lastVisionUpdate = Double.NaN;

    public synchronized void addVisionUpdate(Double timestamp, List<TargetInfo> observations) {


        List<TargetInfo> fieldToVisionTargetInfo = new ArrayList<>();
        if (lastVisionUpdate == timestamp || observations == null || observations.isEmpty()) {
            goal_tracker.maybePruneTracks();
            return;
        }

        final GPose2d turretToLen = new GPose2d(-0.17, 0.0, Rotation2d.fromDegrees(0.0));

        for (TargetInfo target : observations) {
            GPose2d cameraToVT = new GPose2d(target.getPose2d());
            GPose2d fieldToVT = getFieldToTurret(timestamp).transformBy(turretToLen).transformBy(cameraToVT);

            field_to_vt = fieldToVT;

            SmartDashboard.putString("Vision: CameraToVT", new GPose2d(target.getPose2d()).toString());
            fieldToVisionTargetInfo.add(target.TransformedPose(cameraToRobot, new GPose2d(getFieldToVehicle(timestamp))));
        }

        lastVisionUpdate = timestamp;

        // goal_tracker.update(timestamp, fieldToVisionTargetInfo);

    }

    public synchronized void OutputActiveTracks(){
        GoalTracker tracker = goal_tracker;
        List<GoalTracker.TrackReport> reports = tracker.getTracks();

        String printmsg = "Tracks:";
        for(TrackReport tracks : reports){
            
            printmsg += "Pose: " + tracks.field_to_target.toString() + " ID:" + tracks.id + " ";
            // field_to_vt = tracks.field_to_target;
        }

        SmartDashboard.putString("Track Reports", printmsg.toString());
    }

    public synchronized GPose2d getFieldToVT() {
        return field_to_vt;
    }

    public synchronized GTranslation2d getVehicleToVT() {
        GPose2d fieldtoVT = getFieldToVT();
        if (fieldtoVT == null) {
            return null;
        }
        return getFieldToVehicle(Timer.getFPGATimestamp()).inverse().transformBy(fieldtoVT).getTranslation();
    }

    public synchronized GTranslation2d getTurretToVT() {
        GPose2d fieldtoVT = getFieldToVT();
        if (fieldtoVT == null) {
            return null;
        }
        return getFieldToTurret(Timer.getFPGATimestamp()).inverse().transformBy(fieldtoVT).getTranslation();
    }

    public synchronized AimingParameters getAimingParameters(int prev_track_id, double max_track_age,
            GPose2d VT_to_goal_offset) {

        AimingParameters params = new AimingParameters(getFieldToVehicle(Timer.getFPGATimestamp()), field_to_vt,
                lastVisionUpdate);

        return params;
    }

    public void outputTelemetry() {
        SmartDashboard.putString("Robot State: Interpolated Pose", getFieldToVehicle(Timer.getFPGATimestamp()).toString());
        SmartDashboard.putString("Robot State: Pose Est ", field_to_robot_est.getEstimatedPosition().toString());
        try {
            SmartDashboard.putString("Robot State: field to vt", getFieldToVT().toString());
            SmartDashboard.putString("Robot State: Vehicle to VT", getVehicleToVT().toString());
            
        } catch (Exception e) {
            e.printStackTrace();
        }

        SmartDashboard.putString("Robot State: Turret to VT", getTurretToVT().toString());
        // SmartDashboard.putNumber("Robot State: Turret Aim Angle", )
        OutputActiveTracks();
    }

}
