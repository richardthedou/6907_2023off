package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib6907.devices.Omron;
import frc.lib6907.devices.PicoColorSensor;
import frc.lib6907.loops.ILooper;
import frc.lib6907.loops.Loop;
import frc.lib6907.subsystem.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.Intaker.IntakerState;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import frc.robot.util.Util;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;



//TODO: TOP OX
public class Transfer extends Subsystem {

    private Omron bottom_omron;
    private PicoColorSensor top_colorsensor;

    public enum TransferState {
        STOP, INDEXING, PREPARE_TO_SHOOT, DELIVERING, INTAKING, OUTTAKING, HOLDINGBALL;
    }

    private static Transfer mInstance;

    public static Transfer getInstance() {
        if (mInstance == null) {
            mInstance = new Transfer();
        }
        return mInstance;
    }

    private TalonFX mTransfer;
    private TalonSRX mIntakeCentralizer;
    private TalonFX mIntakeFeeder;

    private TransferState mTransferState;
    private PeriodicIO mPeriodicIO;

    private double intake_last_timestamp;

    Color red = new Color(0.5, 0.16, 0.34);
    Color blue = new Color(0.15,0.35, 0.5);

    private Transfer() {

        mPeriodicIO = new PeriodicIO();
        mTransfer = new TalonFX(Constants.Transfer_ID);
        mIntakeCentralizer = new TalonSRX(Constants.Intaker_Centralizer_ID);
        mIntakeFeeder = new TalonFX(Constants.Intaker_Feeder_ID);


        mTransfer.configFactoryDefault();
        mIntakeFeeder.configFactoryDefault();
        mIntakeCentralizer.configFactoryDefault();

        mIntakeFeeder.setNeutralMode(NeutralMode.Coast);
        mIntakeCentralizer.setNeutralMode(NeutralMode.Brake);

        Util.configStatusFrame(mIntakeFeeder);
        Util.configStatusFrame(mTransfer);
        Util.configStatusFrame(mIntakeCentralizer);

        mTransfer.configVoltageCompSaturation(12.0);
        mTransfer.enableVoltageCompensation(true);

        mTransfer.configOpenloopRamp(0.3);
        mIntakeCentralizer.configOpenloopRamp(0.4, 10);

        mTransfer.setInverted(true);
        mIntakeFeeder.setInverted(false);
        mIntakeCentralizer.setInverted(true); //TODO

        mTransfer.configSupplyCurrentLimit(Constants.Transfer_Current_Limit);
        mIntakeFeeder.configSupplyCurrentLimit(Constants.Intaker_Current_Limit);
        mIntakeCentralizer.configSupplyCurrentLimit(Constants.Intaker_Current_Limit);



        top_colorsensor = new PicoColorSensor();
        top_colorsensor.initializeColorMatch(red, blue);

        bottom_omron = new Omron(Constants.Transfer_Bottom_Omron_ID);

        stop();
    }

    private static class PeriodicIO {

        //INPUT
        public double transfer_outperc;
        public Alliance top_color = Alliance.Invalid;
        public double top_proximity;
        public boolean top_connected;
        public Color top_raw_color = new Color(0,0,0);
        public boolean top_detected, top_T_to_F, top_F_to_T;
        private boolean bottom_omron_detected, bottom_omron_T_to_F, bottom_omron_F_to_T;
        private int ball_count;

        

        //OUTPUT
        public double transfer_demand, centralizer_demand,feeder_demand;

    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.top_connected = top_colorsensor.isSensor1Connected();
        mPeriodicIO.top_proximity = top_colorsensor.getProximity1();
        mPeriodicIO.top_raw_color = top_colorsensor.getColor1();
        mPeriodicIO.top_color = mPeriodicIO.top_proximity > 300 ? top_colorsensor.getMatchedColor(mPeriodicIO.top_raw_color): Alliance.Invalid;

        mPeriodicIO.top_detected = mPeriodicIO.top_proximity> 300;
        mPeriodicIO.bottom_omron_detected = bottom_omron.update();
        mPeriodicIO.bottom_omron_T_to_F = bottom_omron.fromTtoF();
        mPeriodicIO.bottom_omron_F_to_T = bottom_omron.fromFtoT();
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Transfer.this) {
                    stop();
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Transfer.this) {
                    mTransferState = determineState(timestamp);

                    switch (mTransferState) {
                        case DELIVERING:
                            handleDelivering(timestamp);
                            break;
                        case INDEXING:
                            handleIndexing(timestamp);
                            break;
                        case PREPARE_TO_SHOOT:
                            handlePreparing(timestamp);
                            break;
                        case INTAKING:
                            handleIntaking(timestamp);
                            break;
                        case STOP:
                            break;
                        case HOLDINGBALL:
                            handleHolding(timestamp);
                            break;
                        case OUTTAKING:
                            handleOuttake(timestamp);
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

    private TransferState determineState(double timestamp) {
        IntakerState mIntakerState = Intaker.getInstance().getState();
        SuperStructureState mSSState = SuperStructure.getInstance().getState();
        if (mIntakerState == IntakerState.HOLDBALL) {
            return TransferState.HOLDINGBALL;
        } else if (mIntakerState == IntakerState.OUTTAKE) {
            return TransferState.OUTTAKING;
        } else if (mSSState == SuperStructureState.SHOOTING) {
            return TransferState.DELIVERING;
        } else if (mIntakerState == IntakerState.INTAKE) {
            intake_last_timestamp = timestamp;
            return TransferState.INTAKING;
        } else if (mIntakerState == IntakerState.INTAKE_FOR_HOLD) {
            intake_last_timestamp = timestamp;
            return TransferState.HOLDINGBALL;
        }else if(Timer.getFPGATimestamp() - intake_last_timestamp < 1.5){
            return TransferState.INTAKING;
        } else if (mSSState == SuperStructureState.AIMING) {
            return TransferState.PREPARE_TO_SHOOT;
        } else {
            return TransferState.INDEXING;
        }
    }

    private void handleIndexing(double timestamp) {
        mPeriodicIO.centralizer_demand = 0.0;
        mPeriodicIO.feeder_demand = 0.0;

        if (mPeriodicIO.top_detected) {
            mPeriodicIO.transfer_demand = 0.0;
        } else if (mPeriodicIO.bottom_omron_detected) {
            mPeriodicIO.transfer_demand = 0.5;
        } else {
            mPeriodicIO.transfer_demand = 0.0;
        }
    }

    private void handleIntaking(double timestamp) {
        mPeriodicIO.centralizer_demand = 0.5;
        mPeriodicIO.feeder_demand = -0.3;

        if (mPeriodicIO.top_detected) {
            mPeriodicIO.transfer_demand = 0.0;
        } else if (mPeriodicIO.bottom_omron_detected) {
            mPeriodicIO.transfer_demand = 0.5;
        } else {
            mPeriodicIO.transfer_demand = 0.0;
        }
        intake_last_timestamp = timestamp;
    }

    private void handlePreparing(double timestamp) {
        if (!mPeriodicIO.top_detected) {
            mPeriodicIO.centralizer_demand = 0.3;
            mPeriodicIO.transfer_demand = 0.45;
            mPeriodicIO.feeder_demand = -0.1;

        } else {
            mPeriodicIO.transfer_demand = 0.0;
            mPeriodicIO.centralizer_demand = 0.3;
            mPeriodicIO.feeder_demand = -0.1;

        }
    }

    private void handleDelivering(double timestamp) {
        mPeriodicIO.centralizer_demand = 0.3;
        mPeriodicIO.transfer_demand = 0.8;
        mPeriodicIO.feeder_demand = -0.1;

    }

    private void handleHolding(double timestamp) {
        mPeriodicIO.centralizer_demand = 0.0;
        mPeriodicIO.transfer_demand = 0.0;
        mPeriodicIO.feeder_demand = 0.0;
    }

    private void handleOuttake(double timestamp) {
        mPeriodicIO.centralizer_demand = -0.3;
        mPeriodicIO.transfer_demand = -0.5;
        mPeriodicIO.feeder_demand = 0.3;
    }

    @Override
    public void writePeriodicOutputs() {
        mTransfer.set(ControlMode.PercentOutput, mPeriodicIO.transfer_demand);
        mIntakeCentralizer.set(ControlMode.PercentOutput, mPeriodicIO.centralizer_demand);
        mIntakeFeeder.set(ControlMode.PercentOutput, mPeriodicIO.feeder_demand);

    }

    @Override
    public void outputTelemetry() {
        if (Constants.kSuperStructureDebug) {
            SmartDashboard.putNumber("Transfer: Centralizer Current", mIntakeCentralizer.getStatorCurrent());
            SmartDashboard.putBoolean("Transfer: top colorsensor connected", mPeriodicIO.top_connected);
            SmartDashboard.putNumber("Transfer: Output Perc", mPeriodicIO.transfer_outperc);
            SmartDashboard.putBoolean("Transfer: bottom omron", mPeriodicIO.bottom_omron_detected);
            SmartDashboard.putString("Transfer: top color", mPeriodicIO.top_color.toString());
            SmartDashboard.putNumber("Tranfer: Top Proximity", mPeriodicIO.top_proximity);
            SmartDashboard.putNumber("Transfer: demand", mPeriodicIO.transfer_demand);
            SmartDashboard.putNumber("Transfer: Centralizer Demand", mPeriodicIO.centralizer_demand);
            SmartDashboard.putString("Transfer: state", mTransferState.toString());
            SmartDashboard.putString("Transfer: Top Raw Color", getColorToString(mPeriodicIO.top_raw_color));
            // SmartDashboard.putString("Transfer: Top OX", mPeriodicIO.top_ox.toString());

        }
    }

    public String getColorToString(Color color) {
        final String FMT_STR = "R: %.3f  G: %.3f  B: %.3f";
        return String.format(FMT_STR, color.red, color.green, color.blue);
    }
    
    @Override
    public void stop() {
        mTransferState = TransferState.STOP;
        mTransfer.set(ControlMode.PercentOutput, 0);
    }

    //not actually useful
    /*
    public void setIndexing() {
        if (mTransferState != TransferState.INDEXING) {
            mTransferState = TransferState.INDEXING;
        }
    }
    
    public void setPrepareToShoot() {
        if (mTransferState != TransferState.PREPARE_TO_SHOOT) {
            mTransferState = TransferState.PREPARE_TO_SHOOT;
        }
    }
    
    public void setDelivering() {
        if (mTransferState != TransferState.DELIVERING) {
            mTransferState = TransferState.DELIVERING;
        }
    }
    
    public void setReverse() {
        if (mTransferState != TransferState.REVERSE) {
            mTransferState = TransferState.REVERSE;
        }
    }
    
    
    */
    
    public boolean isTranferFUll() {
        return mPeriodicIO.ball_count >= 2;
    }

    public boolean getTopOmron() {
        return mPeriodicIO.top_detected;
    }

    public boolean getBottomOmron() {
        return mPeriodicIO.bottom_omron_detected;
    }

    /**
     * 
     * @return 0 for invalid, 1 for same alliance, 2 for opposite alliance
     */
    public int getTopO() {
        if (mPeriodicIO.top_color == DriverStation.getAlliance()) {
            return 1;
        } else if (mPeriodicIO.top_color != Alliance.Invalid) {
            return 2;
        } else {
            return 0;
        }
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

}
