package frc.lib6907.geometry;

import edu.wpi.first.math.geometry.Twist2d;

public class GTwist2d extends Twist2d {
    
    public GTwist2d() {
        super();
    }

    public GTwist2d(double dx, double dy, double dtheta) {
        super(dx, dy, dtheta);
    }

    public GTwist2d(Twist2d twist2d) {
        super(twist2d.dx, twist2d.dy, twist2d.dtheta);
    }

    public GTwist2d scale(double scale) {
        return new GTwist2d(dx * scale, dy * scale, dtheta * scale);
    }

}
