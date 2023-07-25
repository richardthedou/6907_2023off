package frc.lib6907.geometry;

import edu.wpi.first.math.geometry.Rotation2d;

public class GRotation2d extends Rotation2d {
    
    public GRotation2d() {
        super();
    }

    public GRotation2d(Rotation2d r) {
        super(r.getCos(), r.getSin());
    }

    public GRotation2d(double x, double y) {
        super(x, y);
    }

    public GRotation2d(double rad) {
        super(Math.cos(rad), Math.sin(rad));
    }

    public static GRotation2d fromDegrees(double degree) {
        return new GRotation2d(Math.toRadians(degree));
    }

    public GRotation2d nearestPole() {
        double pole_sin = 0.0;
    	double pole_cos = 0.0;
    	if(Math.abs(getCos()) > Math.abs(getSin())){
    		pole_cos = Math.signum(getCos());
    		pole_sin = 0.0;
    	}else{
    		pole_cos = 0.0;
    		pole_sin = Math.signum(getSin());
    	}
    	return new GRotation2d(pole_cos, pole_sin);
    }

    public double distance(Rotation2d other) {
        return inverse().rotateBy(other).getRadians();
    }

    public double distanceDeg(Rotation2d other) {
        return inverse().rotateBy(other).getDegrees();
    }

    public GTranslation2d toTranslation() {
        return new GTranslation2d(getCos(), getSin());
    }

    @Override
    public GRotation2d rotateBy(Rotation2d other) {
        return new GRotation2d(super.rotateBy(other));
    }

    /** 
     * Find the difference between two GRotational 2d (between 0 to pi radian)
    */
    public GRotation2d absDifference(GRotation2d other) {
        return new GRotation2d(Math.abs(this.minus(other).getRadians()));
    }



    public GRotation2d inverse() {
        // return fromRadians(-getRadians())
        return new GRotation2d(getCos(), -getSin());
    }

}
