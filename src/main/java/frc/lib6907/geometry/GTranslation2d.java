package frc.lib6907.geometry;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class GTranslation2d extends Translation2d {

    public GTranslation2d() {
        super();
    }

    public GTranslation2d(double x, double y) {
        super(x, y);
    }

    public GTranslation2d(Translation2d t) {
        super(t.getX(), t.getY());
    }

    public GTranslation2d(double distance, Rotation2d angle) {
        super(distance, angle);
    }

    public static GTranslation2d fromPolar(Rotation2d angle, double distance) {
        return new GTranslation2d(distance, angle);
    }

    public GRotation2d direction() {
        return new GRotation2d(getX(), getY());
    }

    public GTranslation2d scale(double magnitude) {
        return new GTranslation2d(getX() * magnitude, getY() * magnitude);
    }

    public GTranslation2d translateBy(Translation2d other) {
        return new GTranslation2d(getX() + other.getX(), getY() + other.getY());
    }

    public GTranslation2d minus(GTranslation2d other) {
        return this.translateBy(other.inverse());
    }

    @Override
    public GTranslation2d rotateBy(Rotation2d other) {
        return new GTranslation2d(super.rotateBy(other));
    }

    public GTranslation2d inverse() {
        return new GTranslation2d(-getX(), -getY());
    }

    public GTranslation2d copyOf() {
        return scale(1);
    }

    public GTranslation2d scaledTo(double magnitude) {
        return new GTranslation2d(this.scale(magnitude / this.getNorm()));
    }

    /**
     * Find the dot product between two GTranslation2d
     * @param other
     * @return dot product
     */
    public double dotProduct(GTranslation2d other) {
        return this.getX() * other.getX() + this.getY() * other.getY();
    }

    public boolean equals(GTranslation2d other) {
        return other.getX() == this.getX() && other.getY() == this.getY();
    }

    @Override
    public String toString() {
        return String.format("Translation2d(X: %.5f, Y: %.5f)", getX(), getY());
    }
}

