package frc.robot.subsystems;

import java.sql.Driver;
import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.lib6907.control.HolonomicFeedforward;
import frc.lib6907.control.HolonomicMotionProfiledTrajectoryFollower;
import frc.lib6907.control.HolonomicTrajectoryFollower;
import frc.lib6907.devices.gyro.Pigeon;
import frc.lib6907.devices.gyro.Gyro.AngleAxis;
import frc.lib6907.geometry.GPose2d;
import frc.lib6907.geometry.GRotation2d;
import frc.lib6907.geometry.GTranslation2d;
import frc.lib6907.loops.ILooper;
import frc.lib6907.loops.Loop;
import frc.lib6907.subsystem.Subsystem;
import frc.lib6907.util.CircularBuffer;
import frc.lib6907.util.SynchronousPIDF;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.auto.actions.PathFollowAction.PathFollowConfig;
import frc.robot.util.SwerveHeadingController;
import frc.robot.util.Util;
import frc.robot.util.SwerveHeadingController.State;

public class SwerveDrive extends Subsystem {
    public enum SwerveState {
        STOP, MANUAL, TRAJ, AIM, OFFLOAD
    }

    public enum HeadingMethod {
        MANUAL, LIMELIGHT
    }

    public enum AimNode {
        LEFT_CONE(new GTranslation2d(1.0, -0.558)), MID_CUBE(new GTranslation2d(1.0, 0)),
        RIGHT_CONE(new GTranslation2d(1.0, 0.558));

        public GTranslation2d setpointToTag;

        AimNode(GTranslation2d setpointToTag) {
            this.setpointToTag = setpointToTag;
        }
    }

    private final Pigeon mGyro;
    private double gyroAngle = 0.0;
    private boolean gyroOnline = false;
    private double gyroDPS;
    private double pitchAcclerationTimestamp = 0.0;

    private final boolean manualDriveVelocity = true;
    private static final boolean ENABLE_TRANS = true;
    private static final boolean ENABLE_ROTATE = true;

    private static final double NORMAL_WASD_MAX = 0.9;
    private static final double LOW_WASD_MAX = 0.4;
    private static final double LOW_SPIN_MAX = 0.4;

    public SwerveDriveModule front_left, front_right, back_left, back_right;
    private List<SwerveDriveModule> modules;
    private SwerveState mSwerveState = SwerveState.STOP;
    private HeadingMethod mHeadingMethod = HeadingMethod.MANUAL;

    private AimNode aiming_which = AimNode.MID_CUBE; //-1 is left cone, 1 is right cone, 0 is cube

    private static SwerveDrive sInstance;

    public static SwerveDrive getInstance() {
        if (sInstance == null)
            sInstance = new SwerveDrive();
        return sInstance;
    }

    public Orchestra mOrchestra;

    private GTranslation2d manual_drive_vector = new GTranslation2d();
    private double manual_rotate_rate = 0.0;
    private double headingCorrection;
    private double manual_goal_heading = Double.POSITIVE_INFINITY;
    private final GTranslation2d[] rotateUnitVector = new GTranslation2d[] {
            new GTranslation2d(1, GRotation2d.fromDegrees(135)),
            new GTranslation2d(1, GRotation2d.fromDegrees(45)),
            new GTranslation2d(1, GRotation2d.fromDegrees(-45)),
            new GTranslation2d(1, GRotation2d.fromDegrees(-135)) };

    private double lastTimestamp;
    private double lastVelTimestamp = Double.NEGATIVE_INFINITY;
    private Pose2d lastPose;

    private SwerveHeadingController mHeadingController = new SwerveHeadingController();

    private GPose2d mCenterPose;
    private Pose2d mVelMS;
    private SwerveDriveKinematics mKinematics;

    private Trajectory mTrajectory;
    private double mTrajStartTime;
    private HolonomicFeedforward swerveFF = new HolonomicFeedforward(new SimpleMotorFeedforward(0, 0.23, 0));
    private SynchronousPIDF translationPIDF = new SynchronousPIDF(0.5, 0.0, 0.0, 0.0);
    private HolonomicTrajectoryFollower mTrajectoryFollower = new HolonomicMotionProfiledTrajectoryFollower(
            translationPIDF, swerveFF);
    private PathPlannerState autoTargeState = new PathPlannerState();

    public final Field2d m_field = new Field2d();

    //charge station
    private boolean approachCharge = true;
    private double tiltTime = 0.0;
    private double enterTime = 0.0;
    private final double REVERSE_TIME = 0.2;
    private CircularBuffer fieldOrientedPitchDPS = new CircularBuffer(5);
    private double lastPitchTime = 0.0, lastPitchAngle = 0.0;
    private int lastTiltDirectionIsForward = 0; //1 is forward, -1 is backward
    private boolean charge_station_forward = false;

    private SwerveDrive() {
        back_right = new SwerveDriveModule(Constants.Back_Right);
        back_left = new SwerveDriveModule(Constants.Back_Left);
        front_right = new SwerveDriveModule(Constants.Front_Right);
        front_left = new SwerveDriveModule(Constants.Front_Left);

        modules = List.of(front_left, front_right, back_right, back_left);

        mGyro = new Pigeon(20);
        mGyro.setReversed(false);

        mCenterPose = new GPose2d();
        mVelMS = new GPose2d();

        mKinematics = new SwerveDriveKinematics(front_left.getModulePositionToCenter(),
                front_right.getModulePositionToCenter(),
                back_right.getModulePositionToCenter(), back_left.getModulePositionToCenter());

        lastTimestamp = Double.NEGATIVE_INFINITY;
        SmartDashboard.putData("Field", m_field);

    }

    @Override
    public void stop() {
        if (mSwerveState != SwerveState.STOP) {
            mSwerveState = SwerveState.STOP;
        }
        manual_drive_vector = new GTranslation2d();
        modules.forEach((m) -> m.stop());
        mHeadingController.disable();
    }


    public synchronized void setManual(GTranslation2d drive_vector, double rotationInput, double goal_heading,
            boolean lowPower) {
        if (mSwerveState != SwerveState.MANUAL)
            mSwerveState = SwerveState.MANUAL;

        GTranslation2d raw_input = drive_vector.copyOf(); // field oriented

        if (!lowPower) {
            raw_input = raw_input.scale(NORMAL_WASD_MAX);
            rotationInput *= NORMAL_WASD_MAX;
        } else {
            raw_input = raw_input.scale(LOW_WASD_MAX);
            rotationInput *= LOW_SPIN_MAX;
        }

        if (rotationInput != 0) {
            mHeadingController.disable();
        } else if (rotationInput == 0 && manual_rotate_rate != 0) {
            mHeadingController.temporarilyDisable();
        }

        manual_goal_heading = goal_heading;
        manual_rotate_rate = rotationInput;
        manual_drive_vector = raw_input; // last manual drive vector

    }

    public void rotate(double goal_heading) {
        rotate(goal_heading, false, 0);
    }

    public void rotate(double goal_heading, boolean isAuto, double goal_velocity) {
        goal_heading = Util.placeInAppropriate0To360Scope(gyroAngle, goal_heading);

        if (isAuto) {
            mHeadingController.setAutoTarget(goal_heading, goal_velocity, gyroAngle);
            return;
        }

        if (mVelMS.getTranslation().getNorm() < 0.4) {
            mHeadingController.setStationaryTarget(goal_heading, gyroAngle);
        } else {
            mHeadingController.setStabilizationTarget(goal_heading, gyroAngle);
        }
    }

    @Override
    public void writePeriodicOutputs() {
    }

    @Override
    public void readPeriodicInputs() {
        gyroOnline = mGyro.isAlive();
        if (gyroOnline)
            gyroAngle = mGyro.getAngle();
        gyroDPS = mGyro.getFilteredYawDPS();
    }

    @Override
    public void registerEnabledLoops(ILooper in) {
        in.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
                synchronized (SwerveDrive.this) {
                    initializeAll();
                    // stop();
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (SwerveDrive.this) {
                    regularUpdate(timestamp);

                }
            }

            @Override
            public void onStop(double timestamp) {
                synchronized (SwerveDrive.this) {
                    stop();
                }
            }

        });
    }

    private void regularUpdate(double timestamp) {

        if (Double.isFinite(manual_goal_heading) || manual_rotate_rate != 0) {
            mHeadingMethod = HeadingMethod.MANUAL;
        }

        headingCorrection = mHeadingController.updateRotationCorrection(gyroAngle, gyroDPS, timestamp);
        switch (mHeadingMethod) {
            default:
            case MANUAL:
                if (mHeadingController.getState() == State.Autonomous) {
                    break;
                }
                if (Double.isFinite(manual_goal_heading)) {
                    manual_rotate_rate = 0;
                    rotate(manual_goal_heading);
                } else if (mHeadingController.getState() != State.Off
                        && mHeadingController.getState() != State.TemporaryDisable
                        && mHeadingController.getState() != State.Autonomous) {
                    rotate(mHeadingController.getTargetHeading());
                }
                // else use manual_rotate_rate to rotate
                break;
            case LIMELIGHT:
                rotate(RobotState.getInstance().getVehicleToVT().direction().getDegrees());
                break;
        }

        SwerveModuleState[] states = { front_left.getModuleState(), front_right.getModuleState(),
                back_right.getModuleState(), back_left.getModuleState() };
        RobotState.getInstance().updateFieldToVehicleFromOdom(front_left.getUpdateTime(), states);
        mCenterPose = RobotState.getInstance().getFieldToVehicle(timestamp);

        /* 
        // for 23 and later
        SwerveModulePosition[] modulePositions = {front_left.getModulePosition(), front_right.getModulePosition(),
                back_right.getModulePosition(), back_left.getModulePosition() };
        RobotState.getInstance().updateFieldToVehicleFromOdom(front_left.getUpdateTime(), modulePositions);
        mCenterPose = RobotState.getInstance().getFieldToVehicle(timestamp);
        */

        switch (mSwerveState) {
            case MANUAL:
                if (manualDriveVelocity)
                    setVelocityDriveOutput(inverseKinematics(true));
                else
                    setDriveOutput(inverseKinematics(false));
                break;
            case TRAJ:
                updateTrajFollow(timestamp);
                setVelocityDriveOutput(inverseKinematics(true));
            case STOP:
                break;
            case OFFLOAD:
                updateOffloadDriveVector();
                setVelocityDriveOutput(inverseKinematics(true));
                break;
            case AIM:
                updateAimDriveVector();
                setVelocityDriveOutput(inverseKinematics(false));
                break;
            default:
                stop();
                break;
        }
        updateVelMS();
    }
    
    public void updateVelMS(){
        ChassisSpeeds cs = this.mKinematics.toChassisSpeeds(front_left.getModuleState(), front_right.getModuleState(), back_right.getModuleState(), back_left.getModuleState() );
        mVelMS = new GPose2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond, new Rotation2d(cs.omegaRadiansPerSecond));
    }
    
    public GPose2d getRobotPose() {
        return mCenterPose;
    }
    
    private void setDriveOutput(List<GTranslation2d> driveVectors) {
        if(!getAllModuleInitialized()){
            for (int i = 0; i < modules.size(); i++) {
                modules.get(i).setDrivePerc(0);
            }
            return;
        }
    
        for (int i = 0; i < modules.size(); i++) {
            modules.get(i).setDriveVector(driveVectors.get(i), false);
        }
    }
    
    private void setVelocityDriveOutput(List<GTranslation2d> driveVectors) {
        if(!getAllModuleInitialized()){
            for (int i = 0; i < modules.size(); i++) {
                modules.get(i).setDrivePerc(0);
            }
            return;
        }
        for (int i = 0; i < modules.size(); i++) {
            modules.get(i).setDriveVector(driveVectors.get(i), true);
        }
    }
    
    private List<GTranslation2d> inverseKinematics(boolean ensureTrans) {
        List<GTranslation2d> drive_vectors = new ArrayList<GTranslation2d>();
        GTranslation2d transVector = manual_drive_vector.rotateBy(getHeading().inverse()); // field oriented -> robot
                                                                                           // oriented
        double spin = manual_rotate_rate + headingCorrection;
        if (ensureTrans)
            spin = Util.limit(spin, calculateMaxRotationSpeed(spin > 0, transVector));
        double greatestNorm = 0;
        for (int i = 0; i < modules.size(); i++) {
            GTranslation2d sum_vector = new GTranslation2d();
            if (ENABLE_TRANS) {
                sum_vector = transVector.copyOf();
            }
            if (ENABLE_ROTATE) {
                sum_vector = sum_vector.translateBy(rotateUnitVector[i].scale(spin));
            }
            greatestNorm = Math.max(sum_vector.getNorm(), greatestNorm);
            drive_vectors.add(sum_vector.copyOf());
        }
        if (greatestNorm > 1.0) {
            for (GTranslation2d t : drive_vectors) {
                t = t.scale(1.0 / greatestNorm);
            }
        }
    
        // robot oriented
        return drive_vectors;
    }
    
    private double calculateMaxRotationSpeed(boolean turnLeft, GTranslation2d transVector) {
        if (transVector.getNorm() < 0.001)
            return 1.0;
    
        double maxSpeed = 1.0;
        GRotation2d turnDirection;
        double cosAlpha;
        double a = transVector.getNorm();
        double curSpeed;
        for(int i=0; i<modules.size(); i++) {
            turnDirection = modules.get(i).getModulePositionToCenter().direction().rotateBy(GRotation2d.fromDegrees(90*(turnLeft ? 1.0 : -1.0)));
            cosAlpha = transVector.direction().rotateBy(GRotation2d.fromDegrees(180.0)).distance(turnDirection);
            cosAlpha = Math.cos(cosAlpha);
            // high school mathmatics
            curSpeed = Math.sqrt(a*a*cosAlpha*cosAlpha+1-a*a)+a*cosAlpha;
            if (curSpeed < maxSpeed)
                maxSpeed = curSpeed;
        }
    
        return Math.abs(maxSpeed);
    }
    
    private void updateTrajFollow(double timestamp) {
        double curr_t = timestamp - mTrajStartTime;
        double dt = 0.0;
        if (Double.isFinite(lastTimestamp)) {
            dt = timestamp - lastTimestamp;
        }
    
        // field_oriented
        double[] ret = mTrajectoryFollower.calculateDrivePercOutput(mCenterPose, curr_t, dt);
        Translation2d autoDriveOutput = new Translation2d(ret[0], ret[1]);

        rotate(ret[2]);
    
        if (Double.isNaN(autoDriveOutput.getX()))
            autoDriveOutput = new Translation2d(0.0, autoDriveOutput.getY());
        if (Double.isNaN(autoDriveOutput.getY()))
            autoDriveOutput = new Translation2d(autoDriveOutput.getX(), 0.0);
    
        // 开环
        manual_drive_vector = new GTranslation2d(autoDriveOutput.getX(), autoDriveOutput.getY()); // field oriented -> field oriented
    
    }
    
    Translation2d offloadSetpointToVehicle = null;
    
    private boolean updateOffloadDriveVector(){
        
        GTranslation2d setpointToLoad = new GTranslation2d(0,0);
        if(DriverStation.getAlliance() == Alliance.Blue){
            setpointToLoad = new GTranslation2d(-4.0, -0.7);
        }else if(DriverStation.getAlliance() == Alliance.Red){
            setpointToLoad = new GTranslation2d(-4.0, 0.7);
        }
        Translation2d setpointToField = load_trans.plus(setpointToLoad);
        offloadSetpointToVehicle = setpointToField.minus(RobotState.getInstance().getFieldToVehicle(Timer.getFPGATimestamp()).getTranslation());
        GTranslation2d driveOutput = new GTranslation2d();
    
        SmartDashboard.putString("SWERVE: Offload Setpoint TO Vehicle", offloadSetpointToVehicle.toString());
    
        if(offloadSetpointToVehicle.getNorm() < 0.03){
            manual_drive_vector = new GTranslation2d();
            return true;
        }
    
    
        rotate(90);
    
        if(offloadSetpointToVehicle.getNorm() > 1.0){
            driveOutput = (new GTranslation2d(offloadSetpointToVehicle)).scaledTo(0.7);
        }else{
            driveOutput = (new GTranslation2d(offloadSetpointToVehicle)).scale(0.5);
        }
    
        if(driveOutput.getNorm() < 0.06){
            driveOutput = driveOutput.scaledTo(0.06);
    
        }
    
       
        SmartDashboard.putString("Swerve: Go Offload Output", driveOutput.toString());
        manual_drive_vector = driveOutput;
        return false;
    }
    
    public Translation2d getSetpointToVehicleOffLoad(){
        return offloadSetpointToVehicle;
    }
    
    //TODO: check aiming 
    private boolean updateAimDriveVector(){
        GTranslation2d vehicleToVT = RobotState.getInstance().getVehicleToVT();
        GTranslation2d setpointToVT = aiming_which.setpointToTag;
    
        GTranslation2d setpointToVehicle = new GTranslation2d(vehicleToVT.minus(setpointToVT));
    
        SmartDashboard.putString("SWERVE: AIM SETPOINT TO VT", setpointToVehicle.toString());
        if(setpointToVehicle.getNorm() < 0.03){
            manual_drive_vector = new GTranslation2d();
            return true;
        }
        GTranslation2d driveOutput = setpointToVehicle.scale(0.4).rotateBy(getHeading());
        if(driveOutput.getX() > 0.1){
            driveOutput = new GTranslation2d(0.1, driveOutput.getY());
        }
        if(driveOutput.getNorm() < 0.06){
            driveOutput = driveOutput.scaledTo(0.06);
        }
        SmartDashboard.putString("Swerve: Aim Output", driveOutput.toString());
        manual_drive_vector = driveOutput;
        return false;
    }
    
    public void resetOdometry() {
        resetOdometry(new GPose2d());
    }
    
    public void resetOdometry(GPose2d initPose) {
        mCenterPose = initPose;
        RobotState.getInstance().reset(Timer.getFPGATimestamp(), initPose);
    }
    
    public Pose2d getVelMS() {
        return mVelMS;
    }
    
    GTranslation2d load_trans = null;
    
    public void setGoOffload(){
        if(mSwerveState != SwerveState.OFFLOAD){
            load_trans = RobotState.getInstance().getFieldToVehicle(Timer.getFPGATimestamp()).getTranslation();
            mSwerveState = SwerveState.OFFLOAD;
        }
    }
    
    public void setFollowTrajectory(PathFollowConfig config) {
        if (mSwerveState == SwerveState.TRAJ)
            return;
    
        mTrajectory = config.trajectory;
        mTrajectoryFollower.reset();
        mTrajectoryFollower.setTrajectory(mTrajectory);
        mTrajStartTime = Timer.getFPGATimestamp();
        if (config.setInitPose) {
            Pose2d initialPose = ((PathPlannerTrajectory)mTrajectory).getInitialHolonomicPose();
            System.out.println(initialPose.getRotation().getDegrees());
    
            zeroSensors(initialPose);
            mHeadingController.reset(initialPose.getRotation().getDegrees());
        }
        mSwerveState = SwerveState.TRAJ;
    }
    
    public void setAim(AimNode aim_which){
        this.aiming_which = aim_which;
        if(mSwerveState == SwerveState.AIM){
            return;
        }
        mSwerveState = SwerveState.AIM;
    }
    
    public void resetHeading(boolean reset) {
        if (reset == true) {
            resetHeading(0);
        }
    }
    
    public void resetHeading(double angle) {
        zeroSensors(new GPose2d(0, 0, Rotation2d.fromDegrees(angle)));
    }
    
    public GRotation2d getHeading() {
        return GRotation2d.fromDegrees(gyroAngle);
    }
    
    public double getHeadingDegree() {
        return gyroAngle;
    }
    
    public double getDPS() {
        return gyroDPS;
    }
    
    @Override
    public boolean checkSystem() {
        return true;
    }
    
    @Override
    public void zeroSensors() {
        zeroSensors(new GPose2d());
    }
    
    public void zeroSensors(Pose2d startPose) {
        double angle = startPose.getRotation().getDegrees();
        mGyro.reset(angle);
        gyroAngle = angle;
        resetOdometry(new GPose2d(startPose.getTranslation(), GRotation2d.fromDegrees(angle)));
        mHeadingController.reset();
        mVelMS = new Pose2d();
        lastTimestamp = Double.NEGATIVE_INFINITY;
        lastVelTimestamp = Double.NEGATIVE_INFINITY;
    }
    
    public SwerveDriveKinematics getKinematics() {
        return this.mKinematics;
    }
    
    @Override
    public void outputTelemetry() {
        if (SmartDashboard.getBoolean("Swerve Debug", Constants.kSwerveDebug)) {
            SmartDashboard.putNumber("SWERVE: GYRO heading", gyroAngle);
            SmartDashboard.putNumber("SWERVE: GYRO heading DPS", mHeadingController.getTargetHeadingVelocity());
            SmartDashboard.putBoolean("SWERVE: GYRO ONLINE", gyroOnline);
            SmartDashboard.putNumber("SWERVE: GYRO DPS", gyroDPS);
            SmartDashboard.putNumber("SWERVE: heading correction", headingCorrection);
            SmartDashboard.putNumber("SWERVE: GYRO target heading", mHeadingController.getTargetHeading());
            SmartDashboard.putString("SWERVE: Manual Drive Vector", manual_drive_vector.toString());
            SmartDashboard.putString("SWERVE: Center Pose", mCenterPose.toString());
            SmartDashboard.putString("SWERVE: Module 0 State", front_left.getModulePositionToCenter().toString());
            SmartDashboard.putBoolean("SWERVE: initialized", getAllModuleInitialized());
            SmartDashboard.putString("SWERVE: Robot Vel MS", mVelMS.toString());
            SmartDashboard.putNumber("SWERVE: Robot Vel MS Norm", mVelMS.getTranslation().getNorm());
            SmartDashboard.putNumber("SWERVE: Robot Rotation", mGyro.getAngle());
            // SmartDashboard.putNumber("SWERVE: roll", mGyro.getRawAngle(AngleAxis.Roll));
            SmartDashboard.putNumber("SWERVE: pitch", mGyro.getRawAngle(AngleAxis.Pitch));
            SmartDashboard.putNumber("SWERVE: pitch dps", mGyro.getRawDPS(AngleAxis.Pitch));
            SmartDashboard.putString("SWERVE: State", mSwerveState.toString());
    
    
        }
        m_field.setRobotPose(SwerveDrive.getInstance().getRobotPose());
    }
    
    public Pigeon getPigeon() {
        return mGyro;
    }
    
    public SwerveDriveModule[] getModuleArray() {
        return new SwerveDriveModule[] { front_left, front_right, back_right, back_left };
    }
    
    public void initializeAll(){
        front_left.initialize();
        front_right.initialize();
        back_left.initialize();
        back_right.initialize();
    }
    
    public boolean getAllModuleInitialized() {
        return front_left.getInitialized() && front_right.getInitialized() && back_left.getInitialized()
                && back_right.getInitialized();
    }
    
    /* for 2023 and later
    public SwerveModulePosition[] getAllModulePositions(){
        return new SwerveModulePosition[]{front_left.getModulePosition(), front_right.getModulePosition(), back_right.getModulePosition(), back_left.getModulePosition()};
    }
    */

    public Orchestra getOrchestra() {
        if (mOrchestra == null) {
            List<TalonFX> list = new ArrayList<TalonFX>();
            for (SwerveDriveModule i : getModuleArray()) {
                TalonFX[] t = i.getFalcons();
                list.add(t[0]);
                list.add(t[1]);
            }
            mOrchestra = new Orchestra(list);
        }
        return mOrchestra;
    }

    public void putAutoTargetState(PathPlannerState state) {
        Pose2d pose = state.poseMeters;
        m_field.setRobotPose(pose.getX(), pose.getY(), state.holonomicRotation);
    }

}
