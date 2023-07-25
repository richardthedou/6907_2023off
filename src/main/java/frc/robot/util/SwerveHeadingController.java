package frc.robot.util;


import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDrive;
import frc.lib6907.util.SynchronousPIDF;


public class SwerveHeadingController {
	private double targetHeading;
	private double lastTargetVelocity, targetVelocity;
	private double disabledTimestamp;
	private double lastUpdateTimestamp;
	private final double disableTimeLength = 0.2;
	private TrapezoidProfile.Constraints stabilizationConstraints;
	private ProfiledPIDController stabilizationPID;
	private ProfiledPIDController stationaryPID;
	private SimpleMotorFeedforward mFF;
	private final double ALLOWABLE_ERROR=0.5;

	private double stationarykF = 0.0025;
	private double stabilizationkF = 0.0024;

	public enum State{
		Off, Stabilize, TemporaryDisable,Stationary, Autonomous
	}
	private State currentState = State.Off;
	public State getState(){
		return currentState;
	}
	private void setState(State newState){
		currentState = newState;
	}
	
	public SwerveHeadingController() {
		stabilizationConstraints = new TrapezoidProfile.Constraints(180, 300);
		stabilizationPID = new ProfiledPIDController(0.002, 0.0, 0.00, stabilizationConstraints);
		stationaryPID = new ProfiledPIDController(0.003, 0.00, 0.000, stabilizationConstraints);
		mFF = new SimpleMotorFeedforward(0, 0.0017, 0.000);
        targetHeading = 0;
		lastUpdateTimestamp = Timer.getFPGATimestamp();
		stabilizationPID.setTolerance(ALLOWABLE_ERROR);
		stationaryPID.setTolerance(ALLOWABLE_ERROR);
	}
	
	public synchronized void setStabilizationTarget(double target, double current){
		targetHeading = target;
		stabilizationPID.setGoal(targetHeading);
		stationaryPID.reset(current);
		stationaryPID.setGoal(targetHeading);
		setState(State.Stabilize);
	}
	
	public synchronized void setStationaryTarget(double target, double current){
		targetHeading = target;
		stationaryPID.setGoal(targetHeading);
		stabilizationPID.setGoal(targetHeading);
		stabilizationPID.reset(current);
		setState(State.Stationary);
	}

	public synchronized void setAutoTarget(double targetHeading, double targetVelocity, double current){
		this.targetHeading = targetHeading;
		this.lastTargetVelocity = this.targetVelocity;
		this.targetVelocity = targetVelocity;
		stabilizationPID.setGoal(targetHeading);
		stationaryPID.setGoal(targetHeading);
		stationaryPID.reset(current);
		setState(State.Autonomous);
	}

	public void disable(){
		setState(State.Off);
	}
	
	public void temporarilyDisable(){
		setState(State.TemporaryDisable);
		disabledTimestamp = Timer.getFPGATimestamp();
	}
	
	public double getTargetHeading(){
		return targetHeading;
	}

	public double getTargetHeadingVelocity(){
		return targetVelocity;
	}
	
	public synchronized double updateRotationCorrection(double heading, double dps, double timestamp){
		double correction = 0;
		double error = heading - targetHeading;
		double dt = timestamp - lastUpdateTimestamp;
		double acc = 0;
		
		switch (currentState) {
			case Off:
				break;
			case TemporaryDisable:
				targetHeading = heading;
				stabilizationPID.setGoal(targetHeading);
				stabilizationPID.reset(heading);
				stationaryPID.setGoal(targetHeading);
				stationaryPID.reset(heading);
				if (timestamp - disabledTimestamp >= disableTimeLength)
					setState(State.Stabilize);
				break;
			case Stabilize:
				targetVelocity = stabilizationPID.getSetpoint().velocity;
				acc = (targetVelocity - lastTargetVelocity)/dt;
				correction = stabilizationPID.calculate(heading) + mFF.calculate(targetVelocity, acc);
				lastTargetVelocity = targetVelocity;
				break;
			case Stationary:	
				targetVelocity = stationaryPID.getSetpoint().velocity;
				acc = (targetVelocity - lastTargetVelocity)/dt;
				correction = stationaryPID.calculate(heading) +  mFF.calculate(targetVelocity, acc);
				lastTargetVelocity = targetVelocity;

				break;
			case Autonomous:
				// acc = (targetVelocity - lastTargetVelocity)/dt;
				correction = stabilizationPID.calculate(heading) +  targetVelocity * 0.0016;
		}


		lastUpdateTimestamp = timestamp;
        SmartDashboard.putNumber("Heading stabilization position setpoint", stabilizationPID.getSetpoint().position);
		SmartDashboard.putNumber("Heading stabilization velocity setpoint", stabilizationPID.getSetpoint().velocity);
		SmartDashboard.putNumber("Heading stabilization position error", stabilizationPID.getPositionError());

		SmartDashboard.putNumber("Heading statinoary position setpoint", stationaryPID.getSetpoint().position);
		SmartDashboard.putNumber("Heading stationary velocity setpoint", stationaryPID.getSetpoint().velocity);
		SmartDashboard.putNumber("Heading stationary position error", stationaryPID.getPositionError());


		SmartDashboard.putString("Heading State", currentState.toString());
		SmartDashboard.putNumber("Heading Correction", correction);
		return correction;
    }

	public void reset(){
		reset(0);
	}

	public void reset(double measurement){
		stationaryPID.reset(measurement);
		stabilizationPID.reset(measurement);
	}
}
