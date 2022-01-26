package frc.ExternalLib.BionicsLib;

import edu.wpi.first.math.util.Units;

public interface IYawEncoder
// stolen from frc 4909
{
  /**
   * Function that should be called periodically, typically from
   * the using swerve module's `periodic` function.
   */
  default void periodic()
  {
    // no-op
  }

  /**
   * For a relative encoder, this resets the zero point so that distances are
   * measured from the current encoder value. For implementations using an
   * absolute encoder, this function should be left unimplemented so that the
   * default no-op method provided here is used.
   */
  default void setZero()
  {
    // no-op, by default; overridden for implementations with a
    // relative encoder.
  }

  /**
   * Return the distance from the zero point, in degrees
   *
   * @return
   *   The distance from the zero point, in degrees
   */
  double getDistanceDegrees();

  /**
   * Return the distance from the zero point, in radians
   *
   * @return
   *   The distance from the zero point, in radians
   */
  default double getDistanceRadians()
  {
    return Units.degreesToRadians(getDistanceDegrees());
  }

  /**
   * Set the goal point in degrees
   *
   * @param goal
   *   The requested goal, in degrees, to attempt to attain
   */
  //void setGoalDegrees(double goal);

  /**
   * Calculate output based on previous goal provided to
   * `setGoalDegrees`, given the current condition.
   *
   * @return
   *   The calculated output, in range [-1.0, 1.0].
   */
  //double getOutputSignedPercent();

  /**
   * Calculate output based on a new goal, given the current
   * condition. Saves the new goal.
   *
   * @param goal
   *   A new goal, in degrees, to attempt to attain
   *
   * @return
   *   The calculated output, in range [-1.0, 1.0].
   */
  /*default double getOutputSignedPercent(double goalDegrees)
  {
    setGoalDegrees(goalDegrees);
    return getOutputSignedPercent();
  }*/

}