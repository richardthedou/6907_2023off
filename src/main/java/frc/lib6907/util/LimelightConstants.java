package frc.lib6907.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib6907.geometry.GPose2d;
import frc.lib6907.geometry.GRotation2d;

public class LimelightConstants {

    // https://docs.limelightvision.io/en/latest/theory.html#from-pixels-to-angles
    public static final double kHorizontalFOV = 59.6; // degrees
    public static final double kVerticalFOV = 49.7; // degrees
    public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
    public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));

    private int id;
    private String name;
    private String tableName;
    private double height;      // meters
    private GPose2d turretToLens;
    private GRotation2d horizontalPitch;

    public LimelightConstants(int _id, String _name, String _tableName, double _height, Pose2d _turretToLens, Rotation2d _horizontalPitch) {
        this.id = _id;
        this.name = _name;
        this.tableName = _tableName;
        this.height = _height;
        this.turretToLens = new GPose2d(_turretToLens);
        this.horizontalPitch = new GRotation2d(_horizontalPitch);
    }

    public int getId() {
        return id;
    }

    public String getName() {
        return name;
    }

    public String getTableName() {
        return tableName;
    }

    public double getHeight() {
        return height;
    }

    public GPose2d getTurretToLens() {
        return turretToLens;
    }

    public GRotation2d getHorizontalPitch() {
        return horizontalPitch;
    }

}
