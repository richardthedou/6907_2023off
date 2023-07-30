package frc.robot.auto.actions;


import frc.lib6907.auto.Action;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Intaker.IntakerState;
import frc.robot.util.ShootingParameters;

public class ShootManualAction extends Action {

    private ShootingParameters shoot_param;
    /**
     * 
     * @param targetState target Inaker State
     * @param start targetState set time
     * @param end useless
     */
    public ShootManualAction(double start, double end, ShootingParameters param) {
        super(start, end);
        this.shoot_param = param;
    }

    @Override
    public void start() {
        
        SuperStructure.getInstance().setWantShoot(shoot_param);
    
    }

    @Override
    public void run() {
        
    }

    @Override
    public void end() {
        SuperStructure.getInstance().stop();
    }

}
