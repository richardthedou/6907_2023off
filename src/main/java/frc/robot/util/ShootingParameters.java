package frc.robot.util;

public class ShootingParameters {

    private double hood_target_tick;
    private double shooter_target_tickvel;
    private double field_rel_angle;

    public ShootingParameters(double hood_targ, double shooter_targ, double field_angle) {
        this.hood_target_tick = hood_targ;
        this.shooter_target_tickvel = shooter_targ;
        this.field_rel_angle = field_angle;
    }

    public double getHoodTargetTick() {
        return hood_target_tick;
    }

    public double getShooterTargetVel() {
        return shooter_target_tickvel;
    }

    public double getFieldRelAngle() {
        return field_rel_angle;
    }
    
}
