package frc.robot.launcher;

import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.TurretConstants;

@Logged
public class TurretCRTEncoders {
  private final AbsoluteEncoder m_absEncoderMotor;
  private final DutyCycleEncoder m_dutyCycleEncoder;

  public TurretCRTEncoders(AbsoluteEncoder absEncoder, DutyCycleEncoder dutyCycleEncoder) {
    m_absEncoderMotor = absEncoder;
    m_dutyCycleEncoder = dutyCycleEncoder;
  }

  /**
   * The current position of the turret in radians, calculated with the Chinese Remainder Theorem.
   */
  public double getAngleRadians() {
    return getCRTResultAdjusted() * 2.0 * Math.PI / TurretConstants.kLargeGearTeeth;
  }

  /** This adjusts to a decimal number of teeth for more accurate results. */
  private double getCRTResultAdjusted() {
    double teethMotor = getEncoderTeethMotor();
    double adjustment =
        (teethMotor - ((int) teethMotor)) * (2.0 * Math.PI / TurretConstants.kLargeGearTeeth);
    return getCRTResultCorrected() + adjustment;
  }

  /**
   * Corrects for results that go backward over 0, since it would think that it's now at 390 teeth
   * or something similar otherwise. Also, corrects the 0 position to instead be half a rotation.
   */
  private int getCRTResultCorrected() {
    int teeth = getCRTInitResult();
    if (teeth > TurretConstants.kLargeGearTeeth / 2) {
      teeth =
          TurretConstants.kLargeGearTeeth
              - (TurretConstants.kGearTeethMotor * TurretConstants.kGearTeethOther - teeth);
    }
    return (teeth + TurretConstants.kLargeGearTeeth / 2) % TurretConstants.kLargeGearTeeth;
  }

  /**
   * We can use the Chinese Remainder Theorem (CRT) to find the position of the large gear given the
   * position two smaller gears (the number of teeth on the smaller gears must be coprime). For more
   * information on the CRT, see the following sources.
   *
   * <p>Explanation of concept: https://www.geeksforgeeks.org/maths/chinese-remainder-theorem/
   *
   * <p>How to solve: https://www.youtube.com/watch?v=zIFehsBHB8o
   *
   * <p>Background on modular arithmetic (especially modular inverses):
   * https://www.khanacademy.org/computing/computer-science/cryptography/modarithmetic/a/modular-inverses
   */
  private int getCRTInitResult() {
    int teethMotor = (int) getEncoderTeethMotor();
    /*
     * The additional calculations below prevent the situation where one gear is at 1.99 teeth and the
     * other is at 2.00 teeth, but then one becomes an int at 1 tooth and the other is at 2 teeth (or
     * something similar). This is important to avoid because if they were off a little bit, it could
     * entirely throw off the position of the turret overall. Simply rounding both to the nearest int
     * would not solve this problem either, because the same thing could happen if one was at 1.49 and
     * the other was at 1.50 (or a similar situation).
     */
    int teethOther =
        (int) Math.round(getEncoderTeethOther() - (getEncoderTeethMotor() - teethMotor));

    return (TurretConstants.kCRTGearMultiplierMotor * teethMotor
            + TurretConstants.kCRTGearMultiplierOther * teethOther)
        % (TurretConstants.kGearTeethMotor * TurretConstants.kGearTeethOther);
  }

  private double getEncoderTeethMotor() {
    return m_absEncoderMotor.getPosition();
  }

  private double getEncoderTeethOther() {
    return m_dutyCycleEncoder.get() * TurretConstants.kEncoderMaxValue;
  }

  /** This method is only for the purpose of data tracking with Epilogue. */
  private double getRawDutyCycleEncoderPos() {
    return m_dutyCycleEncoder.get();
  }

  public double getVelocity() {
    return m_absEncoderMotor.getVelocity();
  }
}
