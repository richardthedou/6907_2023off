package frc.robot.auto.actions;


import frc.lib6907.auto.Action;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.Intaker.IntakerState;

public class IntakeAction extends Action {

    IntakerState targetState;

    /**
     * 
     * @param targetState target Inaker State
     * @param start targetState set time
     * @param end useless
     */
    public IntakeAction(IntakerState targetState, double start, double end) {
        super(start, end);
        this.targetState = targetState;
    }

    @Override
    public void start() {
        switch(targetState){

            case FEED:
                break;
            case HOME:
                Intaker.getInstance().setHome();
                break;
            case INTAKE:
                Intaker.getInstance().setIntake();
                break;
            default:
                break;

        }
        

    }

    @Override
    public void run() {
        
    }

    @Override
    public void end() {
        Intaker.getInstance().setHome();
    }

}
