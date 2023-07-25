package frc.lib6907.control;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;

public class HolonomicFeedforward {
    private final SimpleMotorFeedforward forwardConstants;
    private final SimpleMotorFeedforward strafeConstants;

    public HolonomicFeedforward(SimpleMotorFeedforward forwardConstants,
                                SimpleMotorFeedforward strafeConstants) {
        this.forwardConstants = forwardConstants;
        this.strafeConstants = strafeConstants;
    }

    public HolonomicFeedforward(SimpleMotorFeedforward translationConstants) {
        this(translationConstants, translationConstants);
    }

    public Translation2d calculateFeedforward(Translation2d velocity, Translation2d acceleration) {
        // We don't use `DrivetrainFeedforwardConstants.calculateFeedforward` because we want to apply kS (the static
        // constant) proportionally based on the rest of the feedforwards.

        double forwardFeedforward = forwardConstants.kv * velocity.getX();
        forwardFeedforward += forwardConstants.ka * acceleration.getX();

        double strafeFeedforward = strafeConstants.kv * velocity.getY();
        strafeFeedforward += strafeConstants.ka * acceleration.getY();

        Translation2d feedforwardVector = new Translation2d(forwardFeedforward, strafeFeedforward);

        // Apply the kS constant proportionally to the forward and strafe feedforwards based on their relative
        // magnitudes
        Translation2d feedforwardUnitVector = feedforwardVector.times(1.0 / feedforwardVector.getNorm());
        forwardFeedforward += Math.copySign(feedforwardUnitVector.getX() * forwardConstants.ks,
                forwardFeedforward);
        strafeFeedforward += Math.copySign(feedforwardUnitVector.getY() * strafeConstants.ks,
                strafeFeedforward);

        return new Translation2d(forwardFeedforward, strafeFeedforward);
    }

}
