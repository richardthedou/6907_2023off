package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import frc.lib6907.geometry.GTranslation2d;
import frc.lib6907.math.InterpolatingDouble;
import frc.lib6907.math.InterpolatingTreeMap;
import frc.robot.subsystems.SwerveDriveModule.SwerveModuleConfigV2;


public class Constants {

    //For Debug
    public static final boolean kSwerveDebug = false;
    public static final boolean kArmDebug = true;
    public static final boolean kIntakerDebug = false;


    public static final double kLooperDt = 0.01;
    public static final boolean kDebugOutput = false;

    public static final SupplyCurrentLimitConfiguration SWERVE_MOTOR_SUPPLY_LIMIT = new SupplyCurrentLimitConfiguration(true, 20, 30, 0.01);
    public static final SupplyCurrentLimitConfiguration GENERAL_MOTOR_SUPPLY_LIMIT = new SupplyCurrentLimitConfiguration(true, 10, 10, 0.01);


    public static final double SWERVE_SIDE_LENGTH = 0.56;
    public static final double SWERVE_WHEEL_DIAMETER = 0.13;

    public static final int DRIVEIO_PORT = 0, OPERATORIO_PORT = 1;
    public static final SwerveModuleConfigV2 Front_Right, Back_Right, Front_Left, Back_Left;

    static {
        final double DRIVE_KP = 0.08,
                DRIVE_KI = 0.0,
                DRIVE_KD = 0.05, //2.0
                DRIVE_KF = 0.045; //0.045
        final double ROTATE_KP = 0.6, 
                ROTATE_KI = 0.0,
                ROTATE_KD = 1.0,
                ROTATE_KF = 0.045;

        Front_Right = new SwerveModuleConfigV2();
        Front_Right.moduleName = "FR";
        Front_Right.moduleID = 2;
        Front_Right.drive_ID = 6;
        Front_Right.rotate_ID = 7;
        Front_Right.rotate_Offset = -1536;
        Front_Right.lightGateID = 2;
        Front_Right.modulePosition = new GTranslation2d(SWERVE_SIDE_LENGTH / 2, -SWERVE_SIDE_LENGTH / 2);
        Front_Right.invertRotate = false;
        Front_Right.invertDrive = false;
        Front_Right.DRIVE_KP = DRIVE_KP;
        Front_Right.DRIVE_KI = DRIVE_KI;
        Front_Right.DRIVE_KD = DRIVE_KD;
        Front_Right.DRIVE_KF = DRIVE_KF;
        Front_Right.ROTATE_KP = ROTATE_KP;
        Front_Right.ROTATE_KI = ROTATE_KI;
        Front_Right.ROTATE_KD = ROTATE_KD;
        Front_Right.ROTATE_KF = ROTATE_KF;

        Back_Right = new SwerveModuleConfigV2();
        Back_Right.moduleName = "BR";
        Back_Right.moduleID = 3;
        Back_Right.drive_ID = 4;
        Back_Right.rotate_ID = 5;
        Back_Right.rotate_Offset = 1536;
        Back_Right.lightGateID = 4;
        Back_Right.modulePosition = new GTranslation2d(-SWERVE_SIDE_LENGTH / 2, -SWERVE_SIDE_LENGTH / 2);
        Back_Right.invertRotate = false;
        Back_Right.invertDrive = false;
        Back_Right.DRIVE_KP = DRIVE_KP;
        Back_Right.DRIVE_KI = DRIVE_KI;
        Back_Right.DRIVE_KD = DRIVE_KD;
        Back_Right.DRIVE_KF = DRIVE_KF;

        Back_Left = new SwerveModuleConfigV2();
        Back_Left.moduleName = "BL";
        Back_Left.moduleID = 0;
        Back_Left.drive_ID = 0;
        Back_Left.rotate_ID = 1;
        Back_Left.rotate_Offset = -1536-6144;
        Back_Left.lightGateID = 3;
        Back_Left.modulePosition = new GTranslation2d(-SWERVE_SIDE_LENGTH / 2, SWERVE_SIDE_LENGTH / 2);
        Back_Left.invertDrive = false;
        Back_Left.invertRotate = false;
        Back_Left.DRIVE_KP = DRIVE_KP;
        Back_Left.DRIVE_KI = DRIVE_KI;
        Back_Left.DRIVE_KD = DRIVE_KD;
        Back_Left.DRIVE_KF = DRIVE_KF;

        Front_Left = new SwerveModuleConfigV2();
        Front_Left.moduleName = "FL";
        Front_Left.moduleID = 1;
        Front_Left.drive_ID = 2;
        Front_Left.rotate_ID = 3;
        Front_Left.rotate_Offset = 1536+6144;
        Front_Left.lightGateID = 0;
        Front_Left.modulePosition = new GTranslation2d(SWERVE_SIDE_LENGTH / 2, SWERVE_SIDE_LENGTH / 2);
        Front_Left.invertDrive = false;
        Front_Left.invertRotate = false;
        Front_Left.DRIVE_KP = DRIVE_KP;
        Front_Left.DRIVE_KI = DRIVE_KI;
        Front_Left.DRIVE_KD = DRIVE_KD;
        Front_Left.DRIVE_KF = DRIVE_KF; //0.0452

    }

   

    //Intaker
    public static final int Intaker_Extend_ID = 17;
    public static final int Intaker_Roller_ID = 18;
    public static final int Intaker_Feeder_ID = 21;
    public static final int Intaker_Centralizer_ID = 12;

    public static final StatorCurrentLimitConfiguration Intaker_Centralizer_Limit = new StatorCurrentLimitConfiguration(true, 40, 40, 0.1);
    public static final SupplyCurrentLimitConfiguration Intaker_Current_Limit = new SupplyCurrentLimitConfiguration(true, 10, 40, 0.2);
    public static final StatorCurrentLimitConfiguration Intaker_Extend_Limit = new StatorCurrentLimitConfiguration(false, 10, 20, 0.1);

    //Transfer
    public static final int Transfer_ID = 33;
    public static final SupplyCurrentLimitConfiguration Transfer_Current_Limit = new SupplyCurrentLimitConfiguration(true, 10, 10, 0.1);
    public static final int Transfer_Top_Omron_ID = 5;
    public static final int Transfer_Bottom_Omron_ID = 8;


    //Turret
    public static final int Turret_ID = 16;
    public static final SupplyCurrentLimitConfiguration Turret_Current_Limit = new SupplyCurrentLimitConfiguration(true, 10, 10, 0.5);
    public static final GTranslation2d kVehicleToTurret = new GTranslation2d(0.16, 0.0); //TODO: vehicle to turret translation


    //Shooter
    public static final int Shooter_Left_ID = 15;
    public static final int Shooter_Right_ID = 27;
    public static final SupplyCurrentLimitConfiguration Shooter_Current_Limit = new SupplyCurrentLimitConfiguration(true, 30, 40, 0.5);

    //Shooter dist -> veltick
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHighShooterMap = new InterpolatingTreeMap<>();
    static {
        kHighShooterMap.put(new InterpolatingDouble(1.8), new InterpolatingDouble(8500.0));
        kHighShooterMap.put(new InterpolatingDouble(3.0), new InterpolatingDouble(9200.0));
        kHighShooterMap.put(new InterpolatingDouble(5.0), new InterpolatingDouble(14000.0));
        // kHighShooterMap.put(new InterpolatingDouble(1.8), new InterpolatingDouble(6750.0));
        // kHighShooterMap.put(new InterpolatingDouble(2.5), new InterpolatingDouble(7200.0));
        // kHighShooterMap.put(new InterpolatingDouble(3.0), new InterpolatingDouble(7625.0));
        // kHighShooterMap.put(new InterpolatingDouble(3.5), new InterpolatingDouble(7750.0));
        // kHighShooterMap.put(new InterpolatingDouble(4.5), new InterpolatingDouble(8250.0));

    }
    
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kLowShooterMap = new InterpolatingTreeMap<>();
    static {
        kLowShooterMap.put(new InterpolatingDouble(2.0), new InterpolatingDouble(4000.0));
        kLowShooterMap.put(new InterpolatingDouble(2.5), new InterpolatingDouble(4250.0));
        kLowShooterMap.put(new InterpolatingDouble(3.0), new InterpolatingDouble(4750.0));
    }

    

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHighHoodMap = new InterpolatingTreeMap<>();
    static {
        //TODO: Test
        // kHighHoodMap.put(new InterpolatingDouble(1.8), new InterpolatingDouble(1000.0));
        // kHighHoodMap.put(new InterpolatingDouble(2.5), new InterpolatingDouble(5000.0));
        // kHighHoodMap.put(new InterpolatingDouble(3.0), new InterpolatingDouble(13000.0));
        // kHighHoodMap.put(new InterpolatingDouble(3.5), new InterpolatingDouble(16000.0));
        // kHighHoodMap.put(new InterpolatingDouble(4.5), new InterpolatingDouble(23000.0));
        kHighHoodMap.put(new InterpolatingDouble(1.8), new InterpolatingDouble(3000.0));
        kHighHoodMap.put(new InterpolatingDouble(3.0), new InterpolatingDouble(12000.0));
        kHighHoodMap.put(new InterpolatingDouble(4.5), new InterpolatingDouble(23000.0));


    }

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kLowHoodMap = new InterpolatingTreeMap<>();
    static {
        //TODO: Test
        kLowHoodMap.put(new InterpolatingDouble(2.0), new InterpolatingDouble(24000.0));
        kLowHoodMap.put(new InterpolatingDouble(3.0), new InterpolatingDouble(24000.0));
    }


    //Hood 
    public static final int Hood_ID = 25;

    //Super Structure
    public static final boolean kSuperStructureDebug = true;



}
