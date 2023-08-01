package frc.robot.devices;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib6907.geometry.GTranslation2d;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.SwerveControllerTest;

public class ControlInput {
    private final SwerveControllerTest mDriver = new SwerveControllerTest(Constants.DRIVEIO_PORT);
    private final XboxController mOperator = new XboxController(Constants.OPERATORIO_PORT);

    private GTranslation2d drive_vector;
    private GTranslation2d operator_left;

    private double drive_targetAngle, drive_rawRate;
    private static ControlInput sInstance;
    private boolean pigeonReset;
    private boolean slowMode;
    private boolean reinitialize;
    private double drive_pov = Double.NaN;


    //Hangar
    private boolean riseLift, lowerLift;

    //shooter
    private boolean autoAimAndShoot;
    private double manualTurret = Double.NaN;
    private boolean fixedShoot;

    //intaker
    private boolean home;
    private boolean intaking;
    private boolean holdBall;
    private boolean outtake;
    private boolean intakeNear;

    //climb 
    private boolean climb_up;
    private boolean climb_down;

    public static ControlInput getInstance() {
        if (sInstance == null)
            sInstance = new ControlInput();
        return sInstance;
    }

    private ControlInput() {
        drive_vector = new GTranslation2d();
        operator_left = new GTranslation2d();
    }

    public synchronized void updateInput() {
        drive_vector = mDriver.getDriveVector(); //left joystick
        drive_rawRate = mDriver.getRawChangeRate(); //trigger (left is CCW, right is CW)
        pigeonReset = mDriver.getPigeonReset(); //start button
        drive_targetAngle = mDriver.getDriveTargetAngle(); //Right joystick
        slowMode = mDriver.getSlowMode(); //Right Bumper
        drive_pov = mDriver.getPOV();  //pov

        //intaker
        home = mDriver.getLeftBumperPressed() || mOperator.getLeftBumperPressed(); 
        intaking = mDriver.getAButtonPressed() || mOperator.getAButtonPressed();
        holdBall = mOperator.getStartButtonPressed();
        outtake = mOperator.getBButtonPressed();
        intakeNear = mOperator.getRightBumperPressed();

        //shoot
        fixedShoot = mOperator.getXButton() || mDriver.getDriver().getXButton();
        autoAimAndShoot = mOperator.getYButton();
        GTranslation2d turret_setpoint = new GTranslation2d(-mOperator.getRightY(), -mOperator.getRightX())
                .rotateBy(SwerveDrive.getInstance().getHeading().inverse());
        if (turret_setpoint.getNorm() > 0.8) {
            manualTurret = turret_setpoint.direction().rotateBy(SwerveDrive.getInstance().getHeading().inverse()) .getDegrees();
        } else {
            manualTurret = Double.NaN;
        }

        operator_left = new GTranslation2d(mOperator.getLeftX(), mOperator.getLeftY());
        
        //climb
        climb_up = mOperator.getPOV() == 0;
        climb_down = mOperator.getPOV() == 180;


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

    public boolean getSlowMode() {
        return slowMode;
    }

    public double getDrivePOV() {
        return drive_pov;
    }
    
    //shooter
    public boolean getVisionShoot() {
        return autoAimAndShoot;
    }

    public boolean getFixedShooter() {
        return fixedShoot;
    }

    public boolean getSouthShoot1() {
        return Math.abs(operator_left.direction().getDegrees() - 180) < 5;
    }

    public boolean getSouthShoot2() {
        return Math.abs(operator_left.direction().getDegrees() - 90) < 5;
    }
    public boolean getSouthShoot3() {
        return Math.abs(operator_left.direction().getDegrees() - 0) < 5;
    }
    
    //intaker
    public boolean getIntaking() {
        return intaking;
    }

    public boolean getHome() {
        return home;
    }

    public boolean getHoldingBall() {
        return holdBall;
    }
    
    public boolean getOuttake() {
        return outtake;
    }
    
    public boolean getIntakeNear() {
        return intakeNear;
    }


    public void outputToSmartDashboard() {
        if (Constants.kDebugOutput) {
            // SmartDashboard.putNumber("DRIVE TARGET ANGLE", drive_targetAngle);
            SmartDashboard.putString("DRIVE VECTOR", drive_vector.toString());
            SmartDashboard.putNumber("ROTATE OUTPUT", getDriveRawChangeRate());
            SmartDashboard.putBoolean("LOW POWER", getSlowMode());
        }

    }

    //climb
    public boolean getClimbUp() {
        return climb_up;
    }

    public boolean getClimbDown() {
        return climb_down;
    }

    //FOR TESTING USE ONLY
    public XboxController getDriver() {
        return mDriver.getDriver();
    }

    public XboxController getOperator() {
        return mOperator;
    }

    public boolean getReinitialize() {
        return reinitialize;
    }
}