package frc.robot.devices;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib6907.geometry.GTranslation2d;
import frc.lib6907.util.EnhancedBoolean;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.SwerveControllerTest;
import frc.robot.util.Util;

public class ControlInput {
    private final SwerveControllerTest mDriver = new SwerveControllerTest(Constants.DRIVEIO_PORT);
    private final XboxController mOperator = new XboxController(Constants.OPERATORIO_PORT);

    private GTranslation2d drive_vector;
    private double drive_targetAngle, drive_rawRate;
    private static ControlInput sInstance;
    private boolean pigeonReset;
    private boolean slowMode;
    private boolean reinitialize;
    private double drive_pov;
    private boolean aim;

    //hangar
    private double hangar_high_perc, hangar_low_perc;
    private boolean hangar_high_stab, hangar_low_stab, hangar_low_stab2;
    
    private boolean hangar_is_ready, hang_mid, hang_high, hang_traverse, hangar_reset;
    private boolean hangar_set_manual = false;

    //shooter
    private boolean autoAimAndShoot, BoeingShoot, launchpadShoot;
    private double manualTurret = Double.NaN;
    private boolean rapidReactMode;
    private double lastDrivePOV = -1;

    //intaker
    private boolean carryBall;
    private EnhancedBoolean intake = new EnhancedBoolean();
    private boolean compressor_digital = true;
    private boolean home;
    private boolean shoot;
    private boolean intaking;
    private boolean vision;
    private boolean offload_stop;
    private boolean load;

    private double arm_perc;
    private double arm_x = 0, arm_y=0, arm_z = 0.15;
    private double arm_vx = 0, arm_vy = 0, arm_vz = 0;

    //operator
    private double operator_left_x, operator_left_y, operator_right_x, operator_right_y, operator_trigger;
    private boolean opOffLoad = false;
    private boolean op_a, op_b, op_x, op_y, op_start, op_rb, op_lb;
    private double op_pov;

    public static ControlInput getInstance() {
        if (sInstance == null)
            sInstance = new ControlInput();
        return sInstance;
    }

    private ControlInput() {
        drive_vector = new GTranslation2d();
    }

    public synchronized void updateInput() {
        drive_vector = mDriver.getDriveVector();
        drive_rawRate = mDriver.getRawChangeRate();
        pigeonReset = mDriver.getPigeonReset();
        drive_targetAngle = mDriver.getDriveTargetAngle();
        slowMode = mDriver.getSlowMode();

        home = mDriver.getLeftBumperPressed();
        intaking = mDriver.getAButtonPressed();
        shoot = mDriver.getDriver().getBButton();
        load = mDriver.getDriver().getXButton();
        // offload = mDriver.getDriver().getYButtonPressed();
        vision = mDriver.getDriver().getYButton();
        offload_stop = mOperator.getRightTriggerAxis() > 0.3;

        aim = mDriver.getDriver().getBackButton();


        drive_pov = mDriver.getPOV();

        //ARM
        operator_left_x = Util.eliminateDeadband(mOperator.getLeftX());
        operator_left_y =  Util.eliminateDeadband(mOperator.getLeftY());
        operator_right_x = Util.eliminateDeadband(mOperator.getRightX());
        operator_right_y = Util.eliminateDeadband(mOperator.getRightY());
        operator_trigger = Util.eliminateDeadband(mOperator.getRightTriggerAxis()-mOperator.getLeftTriggerAxis());

        op_a = mOperator.getAButton();
        op_b = mOperator.getBButton();
        op_x = mOperator.getXButton();
        op_y = mOperator.getYButton();
        op_lb = mOperator.getLeftBumper();
        op_rb = mOperator.getRightBumper();
        op_pov = mOperator.getPOV();
        

        //shoot
        BoeingShoot = mDriver.getDriver().getBButton();
        launchpadShoot = mDriver.getDriver().getYButtonPressed() ? !launchpadShoot : launchpadShoot;
        autoAimAndShoot = mDriver.getVisionAimPressed() ? !autoAimAndShoot : autoAimAndShoot;
        GTranslation2d turret_setpoint = new GTranslation2d(-mOperator.getRightY(), -mOperator.getRightX())
                .rotateBy(SwerveDrive.getInstance().getHeading().inverse());
        if (turret_setpoint.getNorm() > 0.8) {
            manualTurret = turret_setpoint.direction().rotateBy(SwerveDrive.getInstance().getHeading().inverse()) .getDegrees();
        } else {
            manualTurret = Double.NaN;
        }

        if (mDriver.getPOV() == 0 && lastDrivePOV != 0) {
            rapidReactMode = !rapidReactMode;
        }
        lastDrivePOV = mDriver.getPOV();
    }

    public synchronized GTranslation2d getDriveVector() {
        return drive_vector;
    }

    public double getDriveRawChangeRate() {
        return drive_rawRate;
    }

    public double getDriveTargetAngle() {
        return drive_targetAngle;
    }

    public boolean getPigeonReset() {
        return pigeonReset;
    }

    public boolean getAim(){
        return aim;
    }

    public boolean getSlowMode(){
        return slowMode;
    }

    public double getDriverPOV(){
        return drive_pov;
    }

    public double getHangarHighPerc(){
        return hangar_high_perc;
    }

    public double getHangarLowPerc(){
        return hangar_low_perc;
    }

    public boolean getHangarHighStab(){
        return hangar_high_stab;
    }

    public boolean getHangarLowStab(){
        return hangar_low_stab;
    }

    public boolean getHangarLowStab2() {
        return hangar_low_stab2;
    }
    
    public boolean getHangarReady() {
        return hangar_is_ready;
    }

    public boolean getHangMid() {
        return hang_mid;
    }

    public boolean getHangHigh() {
        return hang_high;
    }

    public boolean getHangTraverse() {
        return hang_traverse;
    }

    public boolean getHangReset() {
        return hangar_reset;
    }
    
    public boolean getManualClimb() {
        return hangar_set_manual;
    }

    public void resetManualClimb() {
        hangar_set_manual = false;
    }

    public boolean getShoot() {
        return shoot;
    }
    
    public boolean getLoad(){
        return load;
    }

    public boolean getVisionShoot(){
        return vision;
    }

    public boolean getOffloadStop(){
        return offload_stop;
    }
    
    public boolean getIntaking() {
        return intaking;
    }

    public boolean getHome(){
        return home;
    }

    public double getOpPOV(){
        return op_pov;
    }

    public boolean getOpA(){
        return op_a;
    }
    public boolean getOpB(){
        return op_b;
    }
    public boolean getOpX(){
        return op_x;
    }
    public boolean getOpY(){
        return op_y;
    }

    public boolean getOpLB(){
        return op_lb;
    }

    public boolean getOpRB(){
        return op_rb;
    }
    public boolean getOpStart(){
        return op_start;
    }
    
    public boolean getCompressorDigital() {
        return compressor_digital;
    }


    public double[] getArmPerc() {
        return new double[]{operator_left_x, operator_left_y, operator_right_y, operator_right_x, operator_trigger,0};
    }

    public double getOpTrigger(){
        return arm_perc;
    }


    public void outputToSmartDashboard() {
        if (Constants.kDebugOutput) {
            // SmartDashboard.putNumber("DRIVE TARGET ANGLE", drive_targetAngle);
            SmartDashboard.putString("DRIVE VECTOR", drive_vector.toString());
            SmartDashboard.putNumber("ROTATE OUTPUT", getDriveRawChangeRate());
            SmartDashboard.putBoolean("LOW POWER", getSlowMode());
            SmartDashboard.putBoolean("Hangar Set Manual", hangar_set_manual);
        }

    }

    //FOR TESTING USE ONLY
    public XboxController getDriver() {
        return mDriver.getDriver();
    }

    public XboxController getOperator() {
        return mOperator;
    }

    public boolean getRapidReactMode() {
        return rapidReactMode;
    }

    public boolean getReinitialize() {
        return reinitialize;
    }
}