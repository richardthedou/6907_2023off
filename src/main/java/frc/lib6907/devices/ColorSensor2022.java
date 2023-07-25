package frc.lib6907.devices;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensor2022 {
    private ColorSensorV3 mSensor;
    private ColorMatch mColorMatcher = new ColorMatch();

    private Color red = new Color(1.0, 0.0, 0.0);
    private Color blue = new Color(0.0, 0.0, 1.0);
    private double proximity;
    private Color rawColor = new Color(0.0, 0.0, 0.0);
    private double proxmity_threshold;
    private double confidence_threshold;

    public ColorSensor2022(I2C.Port port) {
        mSensor = new ColorSensorV3(port);
    }

    public void update() {
        proximity = mSensor.getProximity();
        rawColor = mSensor.getColor();
    }
    
    public Alliance getMatchedColor() {

        if (proximity < proxmity_threshold) {
            return Alliance.Invalid;
        }
        try{
            ColorMatchResult result = mColorMatcher.matchClosestColor(rawColor);
            if (result.confidence < confidence_threshold) {
                return Alliance.Invalid;
            }
            if (result.color.equals(blue)) {
                return Alliance.Blue;
            } else if (result.color.equals(red)) {
                return Alliance.Red;
            } else {
                return Alliance.Invalid;
            }
        } catch (Exception e) {
            e.printStackTrace();
            return Alliance.Invalid;
        }
    }
    
    public void initializeColorMatch(Color red, Color blue, double proximity_limit) {
        this.red = red;
        this.blue = blue;
        mColorMatcher.addColorMatch(red);
        mColorMatcher.addColorMatch(blue);
        this.proxmity_threshold = proximity_limit;
    }

    public void setConfidenceLimit(double confidence_threshold) {
        this.confidence_threshold = confidence_threshold;
    }
    
    public Color getRawColor() {
        return rawColor;
    }

    public double getProximity() {
        return proximity;
    }
}
