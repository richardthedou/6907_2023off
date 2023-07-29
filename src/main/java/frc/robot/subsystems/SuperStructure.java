package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib6907.geometry.GRotation2d;
import frc.lib6907.geometry.GTranslation2d;
import frc.lib6907.loops.ILooper;
import frc.lib6907.loops.Loop;
import frc.lib6907.math.InterpolatingDouble;
import frc.lib6907.subsystem.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.ShootingParameters;

public class SuperStructure extends Subsystem {

    public enum SuperStructureState {
        STOP, AIMING, SHOOTING
    }

    private Shooter mShooter = Shooter.getInstance();
    private Turret mTurret = Turret.getInstance();
    private Hood mHood = Hood.getInstance();
    private RobotState mRobotState = RobotState.getInstance();

    private SuperStructureState mWantedState;
    private SuperStructureState mSuperStructureState;

    private boolean mVisionAiming = true;
    private ShootingParameters mShootingParameters = null;
    private boolean shoot_low_hub = false;
    private boolean top_calibrated = false;
    private boolean modeDynamic = false;

    private double nextScheduledShotTime = Double.POSITIVE_INFINITY;
    private final double SAFE_VELTICK = 4000.0; //在此速度以下射球时，shooter可能会卡球

    private static SuperStructure mInstance;

    public static SuperStructure getInstance() {
        if (mInstance == null) {
            mInstance = new SuperStructure();
        }
        return mInstance;
    }

    private SuperStructure() {
        mSuperStructureState = SuperStructureState.STOP;
        mWantedState = SuperStructureState.STOP;
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (SuperStructure.this) {
                    stop();
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (SuperStructure.this) {
                    SuperStructureState newState = null;

                    top_calibrated = mHood.getCalibrated();

                    SuperStructureState wantedState = mWantedState;
                    switch (mSuperStructureState) {
                        case STOP:
                            newState = handleStop(wantedState);
                            break;
                        case AIMING:
                            newState = handleAiming(wantedState, timestamp);
                            break;
                        case SHOOTING:
                            newState = handleShooting(wantedState);
                            break;
                        default:
                            break;

                    }

                    if (newState != mSuperStructureState) {
                        System.out.println(timestamp + ": Super Structure change state: " + mSuperStructureState + " ->"
                                + newState);
                        mSuperStructureState = newState;
                    }

                    switch (mSuperStructureState) {
                        case STOP:
                            writeStopDesiredState(timestamp);
                            break;
                        case AIMING:
                            writeAimingDesiredState(timestamp);
                            break;
                        case SHOOTING:
                            writeShootDesiredState(timestamp);
                            break;
                        default:
                            break;

                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }

        });
    }

    private SuperStructureState handleStop(SuperStructureState wantedState) {
        switch (wantedState) {
            default:
            case STOP:
                return SuperStructureState.STOP;
            case AIMING:
            case SHOOTING:
                if (!top_calibrated) {
                    return SuperStructureState.STOP;
                } else {
                    return SuperStructureState.AIMING;
                }
        }
    }

    private SuperStructureState handleAiming(SuperStructureState wantedState, double timestamp) {
        switch (wantedState) {
            case STOP:
                return SuperStructureState.STOP;
            default:
            case AIMING:
                return SuperStructureState.AIMING;
            case SHOOTING:
                //regarding scheduled shots
                if (timestamp > nextScheduledShotTime) {
                    if (mShooter.getSpinningVeltick() > SAFE_VELTICK) {
                        nextScheduledShotTime = Double.POSITIVE_INFINITY;
                        return SuperStructureState.SHOOTING;
                    } else {
                        nextScheduledShotTime += 1.0;
                        return SuperStructureState.AIMING;
                    }
                } else if (Double.isFinite(nextScheduledShotTime) && timestamp < nextScheduledShotTime) {
                    return SuperStructureState.AIMING;
                }
                
                //if no scheduled shot
                if (isReadyToShoot() && (!lowhub() || !getLowHubShootBlocked())) {
                    return SuperStructureState.SHOOTING;
                } else {
                    return SuperStructureState.AIMING;
                }
        }
    }

    private SuperStructureState handleShooting(SuperStructureState wantedState) {
        switch (wantedState) {
            case AIMING:
                return SuperStructureState.AIMING;
            case SHOOTING:
            default:
                if (!isReadyToShoot()) {
                    return SuperStructureState.AIMING;
                }else{
                    return SuperStructureState.SHOOTING;
                }
            case STOP:
                return SuperStructureState.STOP;
        }
    }

    private void writeStopDesiredState(double timestamp) {
        mHood.stop();
        mShooter.setIdle();
        // Turret not stoped because of manual adjustment and lock field rel direction
    }

    private boolean tolerable = false;

    private void aimto() {
        if (mVisionAiming) {
            GTranslation2d turretToVT = mRobotState.getTurretToVT();
            GTranslation2d swerveVel = new GTranslation2d(SwerveDrive.getInstance().getVelMS().getTranslation());
            // if (!modeDynamic) {
            double dist = turretToVT.getNorm();
            double angle = turretToVT.direction().rotateBy(RobotState.getInstance().getVehicleToTurret(Timer.getFPGATimestamp()).getRotation()).getDegrees();
            SmartDashboard.putNumber("SS SHOOT DIST", dist);
            SmartDashboard.putNumber("SS SHOOT ANGLE", angle);
            mShootingParameters = new ShootingParameters(
                    Constants.kHighHoodMap.getInterpolated(new InterpolatingDouble(dist)).value,
                    Constants.kHighShooterMap.getInterpolated(new InterpolatingDouble(dist)).value,
                    angle + SwerveDrive.getInstance().getHeadingDegree());
           
            // } else if (modeDynamic){

                
                // GTranslation2d targetVel = swerveVel.rotateBy(SwerveDrive.getInstance().getHeading().inverse())
                //         .inverse(); // velocity in vehicle reference
                // double guess_t_low = 0.6, guess_t_high = 2.0;
                // double mid = (guess_t_low + guess_t_high) / 2;
                // double error = Double.POSITIVE_INFINITY;
                // tolerable = false;
                // for (int i = 0; i < 5; i++) {
                //     mid = (guess_t_low + guess_t_high) / 2;
                //     error = get_target_error(turretToVT, targetVel, mid);
                //     if (error > 0.4) {
                //         guess_t_high = mid;
                //     } else if (error < -0.4) {
                //         guess_t_low = mid;
                //     } else {
                //         tolerable = true;
                //         SmartDashboard.putNumber("HUIHUI: tolerableTime", mid);
                //         break;
                //     }
                // }


                // SmartDashboard.putBoolean("HUIHUI: Tolerable", tolerable);

                // if (tolerable) {
                //     double shoot_dist = Constants.kShootTimeToDistMap
                //             .getInterpolated(new InterpolatingDouble(mid)).value;
                //     GRotation2d angle = RobotState.getInstance().getTurretToVT().translateBy(targetVel.scale(mid)).direction()
                //             .rotateBy(SwerveDrive.getInstance().getHeading()); // field rel
                //     SmartDashboard.putNumber("HUIHUI: shoot_dist", shoot_dist);
                //     SmartDashboard.putNumber("HUIHUI: shoot_angle", angle.getDegrees());

                //     mShootingParameters = new ShootingParameters(
                //             Constants.kHighHoodMap.getInterpolated(new InterpolatingDouble(shoot_dist)).value,
                //             Constants.kHighShooterMap.getInterpolated(new InterpolatingDouble(shoot_dist)).value,
                //             angle.getDegrees());
                // }
            // }
        }
        // if (Transfer.getInstance().getTopO() == 2) {
        //     mHood.setHood(30000);
        //     mShooter.setShootVel(5000);
        //     mTurret.setFieldAngle(180,0);
        // } else {
        mHood.setHood(mShootingParameters.getHoodTargetTick());
        mShooter.setShootVel(mShootingParameters.getShooterTargetVel());
        mTurret.setFieldAngle(mShootingParameters.getFieldRelAngle(),0);
        // }
    }

    // private double get_target_error(GTranslation2d turretToVT, GTranslation2d vehicleVel, double guessTime) {
    //     SmartDashboard.putNumberArray("HUIHUI: TurretToVT", new double[]{turretToVT.getX(),turretToVT.getY()});
    //     SmartDashboard.putNumberArray("HUIHUI: VehicleVel", new double[] {vehicleVel.getX(), vehicleVel.getY()});
    //     double calculated = Math.pow(turretToVT.getNorm(), 2) + Math.pow(vehicleVel.getNorm() * guessTime, 2)
    //             + 2 * guessTime * turretToVT.dotProduct(vehicleVel);
    //     double fromMap = Math
    //             .pow(Constants.kShootTimeToDistMap.getInterpolated(new InterpolatingDouble(guessTime)).value, 2);
    //     return fromMap - calculated;
    // }

    private void writeAimingDesiredState(double timestamp) {
        aimto();
    }

    private void writeShootDesiredState(double timestamp) {
        aimto();
    }

    @Override
    public synchronized void stop() {
        mWantedState = SuperStructureState.STOP;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        if (Constants.kSuperStructureDebug) {
            SmartDashboard.putString("Superstructure: State", mSuperStructureState.toString());
            SmartDashboard.putString("Superstructure: Wanted State", mWantedState.toString());
            SmartDashboard.putBoolean("Superstructure: Top calibrate", top_calibrated);
            if (mShootingParameters != null) {
                SmartDashboard.putNumber("SHOOTPARAM SHOOTER", mShootingParameters.getShooterTargetVel());
            }
            SmartDashboard.putBoolean("SS: ready to shoot", isReadyToShoot());
        }
    }

    public synchronized void setWantShoot(ShootingParameters shoot_param) {
        if (shoot_param == null)
            return;
        mWantedState = SuperStructureState.SHOOTING;
        this.mShootingParameters = shoot_param;
        this.mVisionAiming = false;
    }

    public synchronized void setWantVisionAim(boolean low_hub) {
        mWantedState = SuperStructureState.AIMING;
        this.mVisionAiming = true;
        this.shoot_low_hub = low_hub;
    }

    //schedule shoot in autonomous mode

    /** 
     * @apiNote set the wanted state to AIMING and schedule the shoot time. This shot will be forced (no matter the shoot_param is reached or not) because it's used in auto.
    * @param shoot_timestamp time in seconds after start of auto
    */
    public synchronized void scheduleNextShoot(ShootingParameters shoot_param, double shoot_timestamp) {
        nextScheduledShotTime = shoot_timestamp;
        setWantShoot(shoot_param);
    }

    public synchronized void setWantShootVision() {
        setWantShootVision(false);
    }

    public synchronized void setWantShootVision(boolean low_hub) {
        mWantedState = SuperStructureState.SHOOTING;
        this.mVisionAiming = true;
        this.shoot_low_hub = low_hub;
    }

    public boolean getLowHubShootBlocked() {
        return shoot_low_hub ? (mTurret.getTurretDegreeFieldRel() - 66 + 5) % 90 < 10 : false;
        // return false;
    }

    public boolean isReadyToShoot() {
        if (!modeDynamic) {
            return mShooter.getVelocityReached()
                    && mHood.isMagicInPosition()
                    && mTurret.isTurretInPosition() && SwerveDrive.getInstance().getVelMS().getTranslation().getNorm() < 0.4 && SwerveDrive.getInstance().getDPS() < 5;
        } else if (mVisionAiming){
            return mShooter.getVelocityReached()
                    && mHood.isMagicInPosition()
                    && mTurret.isTurretInPosition() && tolerable;
        } else{
            return mShooter.getVelocityReached()
            && mHood.isMagicInPosition()
                    && mTurret.isTurretInPosition();
        }
    }

    public SuperStructureState getState() {
        return mSuperStructureState;
    }

    public boolean lowhub(){
        return shoot_low_hub;
    }

}
