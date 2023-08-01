// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib6907.auto.AutoMode;
import frc.lib6907.auto.AutoRunner;
import frc.lib6907.geometry.GPose2d;
import frc.lib6907.loops.Looper;
import frc.lib6907.subsystem.Subsystem;
import frc.lib6907.util.SubsystemManager;
import frc.robot.auto.AutoModeSelector;
import frc.robot.devices.ControlInput;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveDriveModule;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.VisionProcessor;
import frc.robot.util.ShootingParameters;

import com.pathplanner.lib.server.PathPlannerServer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot dsocumentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	/**
	 * This function is run when the robot is first started up and should be used
	 * for any
	 * initialization code.
	 */
	double target_angle = 0;
	SwerveDriveModule testModule;

	private final Looper mEnabledLooper = new Looper();
	private final Looper mDisabledLooper = new Looper();
	private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

	private final RobotState mRobotState = RobotState.getInstance();

	// list of subsystems
	private final SwerveDrive mSwerveDrive = SwerveDrive.getInstance();
	private final VisionProcessor mProcessor = VisionProcessor.getInstance();
	private final Intaker mIntaker = Intaker.getInstance();
	private final LED mLED = LED.getInstance();
	private final Hood mHood = Hood.getInstance();
	private final Shooter mShooter = Shooter.getInstance();
	private final SuperStructure mSuperStructure = SuperStructure.getInstance();
	private final Transfer mTransfer = Transfer.getInstance();
	private final Turret mTurret = Turret.getInstance();
	private final Limelight mLL = Limelight.getTheOnlyLL();
	private final Climb mClimb = Climb.getInstance();

	private final ControlInput mControlInput = ControlInput.getInstance();

	private AutoModeSelector mAutoModeSelector;
	private AutoMode mAutoMode;
	private AutoRunner mAutoRunner;

	private final SendableChooser<SwerveDriveModule> testModuleChooser = new SendableChooser<>();

	public double raceStartTimestamp = Double.NaN;

	@Override
	public void robotInit() {

		// CameraServer.startAutomaticCapture();
		PathPlannerServer.startServer(5811);
		mSubsystemManager.setSubsystems(new Subsystem[] {
				mSwerveDrive,
				mHood,
				mProcessor,
				mIntaker,
				mLED,
				mShooter,
				mTurret,
				mTransfer,
				mSuperStructure,
				mLL,
				mClimb
		},
				mSwerveDrive.getModuleArray());

		mSubsystemManager.registerEnabledLoops(mEnabledLooper);
		mSubsystemManager.registerDisabledLoops(mDisabledLooper);

		mAutoModeSelector = AutoModeSelector.getInstance();
		mRobotState.reset(Timer.getFPGATimestamp(), new GPose2d());

		for (SwerveDriveModule m : mSwerveDrive.getModuleArray()) {
			testModuleChooser.addOption(m.getModuleName(), m);
		}

		SmartDashboard.putData("Module", testModuleChooser);

		System.out.println("Robot init finished");
		mSwerveDrive.zeroSensors();

		for (int i = 0; i < 6; i++) {
			SmartDashboard.putNumber("Tuning P", 0);
			SmartDashboard.putNumber("Tuning I", 0);
			SmartDashboard.putNumber("Tuning D", 0);
		}

		SmartDashboard.putBoolean("Intake Debug", Constants.kIntakerDebug);
		SmartDashboard.putBoolean("Arm Debug", Constants.kArmDebug);
		SmartDashboard.putBoolean("Swerve Debug", Constants.kSwerveDebug);
		SmartDashboard.putBoolean("Super Structure Debug", Constants.kSuperStructureDebug);

	}

	@Override
	public void robotPeriodic() {
		mSubsystemManager.outputToSmartDashboard();
		mControlInput.outputToSmartDashboard();
		mRobotState.outputTelemetry();

	}

	public void autonomousInit() {
		mDisabledLooper.stop();
		mEnabledLooper.start();

		mAutoMode = mAutoModeSelector.getSelected();
		mAutoRunner = new AutoRunner(mAutoMode.getActionList());

		// mSwerveDrive.zeroSensors(new GPose2d(mAutoMode.getStartPos()));
		mSwerveDrive.initializeAll();

		raceStartTimestamp = Timer.getFPGATimestamp();
		mAutoRunner.initAuto(raceStartTimestamp);
		mAutoMode.initAuto();
	}

	@Override

	public void autonomousPeriodic() {
		mAutoRunner.updatePeriodic(Timer.getFPGATimestamp());
	}

	@Override
	public void teleopInit() {
		mDisabledLooper.stop();
		mEnabledLooper.start();
		// SwerveDrive.getInstance().initializeAll();

	}

	boolean sucking = false;

	@Override
	public void teleopPeriodic() {
		try {
			mControlInput.updateInput();

			// if(mControlInput.getDriverPOV() == 90){
			// lastAimNode = AimNode.RIGHT_CONE;
			// }else if(mControlInput.getDriverPOV() == 0){
			// lastAimNode = AimNode.MID_CUBE;
			// }else if(mControlInput.getDriverPOV() == 270){
			// lastAimNode = AimNode.LEFT_CONE;
			// }

			// Swerve
			// if(mControlInput.getDriverPOV() == 0){
			// mSwerveDrive.setCharge(true);
			// }else if(mControlInput.getDriverPOV() == 180){
			// mSwerveDrive.setCharge(false);
			// if(mControlInput.getAim()){
			// mSwerveDrive.setAim(lastAimNode);
			// }else

			//pigeon fused heading and robot heading has 180 degree difference
			double targetAngle = mControlInput.getDriveTargetAngle()+180;

			if(mControlInput.getDrivePOV() == 180){
				targetAngle = 180;
			} else if (mControlInput.getDrivePOV() == 0) {
				targetAngle = 0;
			}


			mSwerveDrive.setManual(mControlInput.getDriveVector().scale(0.75), mControlInput.getDriveRawChangeRate() * 0.7,
					targetAngle, mControlInput.getVisionShoot() ? true : mControlInput.getSlowMode());

			if (mControlInput.getPigeonReset()) { // driver start
				System.out.println("sensor zeroed");
				mSwerveDrive.zeroSensors();
				mSwerveDrive.rotate(0);
				SwerveDrive.getInstance().initializeAll();
			}

			// Intake
			if (mControlInput.getIntaking()) { // driver a
				mIntaker.setIntake();
			} else if(mControlInput.getIntakeNear()){
				mIntaker.setIntakeNear();
			}else if (mControlInput.getHome() || mControlInput.getOperator().getXButtonReleased()) {// driver left bumper
				mIntaker.setHome();
			} else if (mControlInput.getOuttake()) {
				mIntaker.setOuttake();
			} else if (mControlInput.getHoldingBall()) {
				mIntaker.setHoldBall();
			}

			if (mControlInput.getFixedShooter()) {
				mSuperStructure.setWantShoot(new ShootingParameters(8000, 5500, 0));
			} else if (mControlInput.getVisionShoot()) {
				mSuperStructure.setWantShootVision();
			} else if (mControlInput.getSouthShoot1()){
				mSuperStructure.setWantShoot(new ShootingParameters(30000, 5500, 180));
			}else if (mControlInput.getSouthShoot2()){
				mSuperStructure.setWantShoot(new ShootingParameters(30000, 7000, 180));
			}else if (mControlInput.getSouthShoot3()){
				mSuperStructure.setWantShoot(new ShootingParameters(30000, 8500, 180));
			}
			
			else {
				mSuperStructure.stop();
			}

			// Climb
			if (mControlInput.getClimbUp()) {
				mClimb.setClimb(69500);
			} else if (mControlInput.getClimbDown()) {
				mClimb.setClimb(500);
			}

		} catch (Exception ex) {
			ex.printStackTrace();
		}
	}


	public void disabledInit() {
		System.out.println("Disabled Loop running");
		mDisabledLooper.start();
		mEnabledLooper.stop();
	}

	public void disabledPeriodic() {

	}

	public void testInit() {
		mDisabledLooper.stop();
		mEnabledLooper.start();

		testModule = testModuleChooser.getSelected();

		// GPose2d tempTestInitPose = new GPose2d(-3.0, 0, GRotation2d.fromDegrees(0));

		// mRobotState.reset(Timer.getFPGATimestamp(), tempTestInitPose);
		// mSwerveDrive.resetHeading(tempTestInitPose.getRotation().getDegrees());
		// mSwerveDrive.resetOdometry(tempTestInitPose);
		// testFalcon3.setSelectedSensorPosition(0);
	}

	public void testPeriodic() {

		// if(!mControlInput.getOperator().getAButton())
		// srx.set(ControlMode.PercentOutput, 0.7);
		// else
		// srx.set(ControlMode.PercentOutput, -0.7);
		XboxController op = mControlInput.getOperator();
		mControlInput.updateInput();
	}
}
