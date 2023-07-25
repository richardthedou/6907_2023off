package frc.lib6907.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class GPose2d extends Pose2d {
    
    public GPose2d() {
        super();
    }

    public GPose2d(double x, double y, Rotation2d rotation) {
        super(x, y, rotation);
    }

    public GPose2d(Translation2d translation, Rotation2d rotation) {
        this(translation.getX(), translation.getY(), rotation);
    }

    public GPose2d(Pose2d pose) {
        this(pose.getTranslation(), pose.getRotation());
    }

    public GPose2d inverse() {
        GRotation2d r_inv = new GRotation2d(getRotation()).inverse();
        return new GPose2d(new GTranslation2d(getTranslation()).inverse().rotateBy(r_inv), r_inv);
    }

    public GPose2d transformBy(final Pose2d other) {
        return new GPose2d(new GTranslation2d(getTranslation()).translateBy(other.getTranslation().rotateBy(getRotation())),
                getRotation().rotateBy(other.getRotation()));
    }

    public GPose2d addTranslation(final Translation2d t2d) {
        return new GPose2d(getTranslation().plus(t2d), getRotation());
    }

    @Override
    public GTranslation2d getTranslation() {
        return new GTranslation2d(super.getTranslation());
    }

}
