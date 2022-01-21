package frc.ExternalLib.JackInTheBotLib.util;

import frc.ExternalLib.JackInTheBotLib.math.Vector2;
import frc.ExternalLib.NorthwoodLib.MathWrappers.NWTranslation2d;

public class HolonomicFeedforward {
    private final DrivetrainFeedforwardConstants forwardConstants;
    private final DrivetrainFeedforwardConstants strafeConstants;

    public HolonomicFeedforward(DrivetrainFeedforwardConstants forwardConstants,
                                DrivetrainFeedforwardConstants strafeConstants) {
        this.forwardConstants = forwardConstants;
        this.strafeConstants = strafeConstants;
    }

    public HolonomicFeedforward(DrivetrainFeedforwardConstants translationConstants) {
        this(translationConstants, translationConstants);
    }

    public NWTranslation2d calculateFeedforward(NWTranslation2d velocity, NWTranslation2d acceleration) {
        // We don't use `DrivetrainFeedforwardConstants.calculateFeedforward` because we want to apply kS (the static
        // constant) proportionally based on the rest of the feedforwards.

        double forwardFeedforward = forwardConstants.getVelocityConstant() * velocity.getX();
        forwardFeedforward += forwardConstants.getAccelerationConstant() * acceleration.getX();

        double strafeFeedforward = strafeConstants.getVelocityConstant() * velocity.getY();
        strafeFeedforward += strafeConstants.getAccelerationConstant() * acceleration.getY();

       NWTranslation2d feedforwardVector = new NWTranslation2d(forwardFeedforward, strafeFeedforward);

        // Apply the kS constant proportionally to the forward and strafe feedforwards based on their relative
        // magnitudes
        NWTranslation2d feedforwardUnitVector = feedforwardVector.normal();
        forwardFeedforward += Math.copySign(feedforwardUnitVector.getX() * forwardConstants.getStaticConstant(),
                forwardFeedforward);
        strafeFeedforward += Math.copySign(feedforwardUnitVector.getY() * strafeConstants.getStaticConstant(),
                strafeFeedforward);

        return new NWTranslation2d(forwardFeedforward, strafeFeedforward);
    }

    public DrivetrainFeedforwardConstants getForwardConstants() {
        return forwardConstants;
    }

    public DrivetrainFeedforwardConstants getStrafeConstants() {
        return strafeConstants;
    }
}
