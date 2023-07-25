package frc.robot.util;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Util {

    public static final double DEADBAND = 0.05;

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    public static double eliminateDeadband(double input) {
        if (Math.abs(input) < DEADBAND)
            return 0.0;
        else
            return input;
    }

    public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

    public static double boundAngle0to360Degrees(double angle) {
        if (angle < 0)
            return angle % 360.0 + 360.0;
        else
            return angle % 360.0;
    }

    // public static double limitTurret(double turret_degrees) {
    //     if (turret_degrees < Constants.kTurretConstants.kMinUnitsSoftLimit) {
    //         turret_degrees += 360.0;
    //     }
    //     if (turret_degrees > Constants.kTurretConstants.kMaxUnitsSoftLimit) {
    //         turret_degrees -= 360.0;
    //     }

    //     return Util.limit(turret_degrees, Constants.kTurretConstants.kMinUnitsSoftLimit, Constants.kTurretConstants.kMaxUnitsSoftLimit);
    // }

    public static boolean shouldReverse(double goalAngle, double currentAngle) {
        goalAngle = boundAngle0to360Degrees(goalAngle);
        currentAngle = boundAngle0to360Degrees(currentAngle);
        double reversedAngle = boundAngle0to360Degrees(currentAngle + 180);
        double angleDifference = Math.abs(goalAngle - currentAngle);
        double reversedAngleDifference = Math.abs(goalAngle - reversedAngle);
        angleDifference = (angleDifference > 180) ? 360 - angleDifference : angleDifference;
        reversedAngleDifference = (reversedAngleDifference > 180) ? 360 - reversedAngleDifference
                : reversedAngleDifference;
        return reversedAngleDifference < angleDifference;
    }

    public static void configStatusFrame(BaseTalon talon) {
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 1000, 100);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 1000, 100);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 1000, 100);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 1000, 100);
        // talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 1000, 100);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 1000, 100);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 1000, 100);

        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 100, 100);
        if (talon instanceof TalonFX)
            talon.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 1000, 100);
    }

    public static void configStatusFrameSlave(BaseTalon talon) {
        configStatusFrame(talon);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1000, 100);
    }

}
