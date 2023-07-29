package frc.robot.auto.actions;


import frc.lib6907.auto.Action;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.Intaker.IntakerState;

public class OuttakeAction extends Action {

    /**
     * 
     * @param targetState target Inaker State
     * @param start targetState set time
     * @param end useless
     */
    public OuttakeAction(double start, double end) {
        super(start, end);
    }

    @Override
    public void start() {
        
        Intaker.getInstance().setOuttake();
    
    }

    @Override
    public void run() {
        
    }

    @Override
    public void end() {
        Intaker.getInstance().setHome();
    }

}
