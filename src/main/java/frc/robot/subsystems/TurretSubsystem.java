package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TurretConstants;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

@Logged
public class TurretSubsystem extends SubsystemBase {
  private final SparkMax m_turretMotor =
      new SparkMax(TurretConstants.kTurretCANId, MotorType.kBrushless);
  private final AbsoluteEncoder m_encoder19Teeth = m_turretMotor.getAbsoluteEncoder();
  private final SparkMax m_turretMotor2 =
      new SparkMax(TurretConstants.kTurretCANId2, MotorType.kBrushless);
  private final AbsoluteEncoder m_encoder21Teeth = m_turretMotor2.getAbsoluteEncoder();

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          TurretConstants.kSTurret, TurretConstants.kVTurret, TurretConstants.kATurret);
  private double ff = 0.0;

  private final PIDController feedback =
      new PIDController(
          TurretConstants.kPTurret, TurretConstants.kITurret, TurretConstants.kDTurret);
  private double fb = 0.0;

  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private final TrapezoidProfile profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              TurretConstants.kMaxVelocityTurret, TurretConstants.kMaxAccelerationTurret));

  private Supplier<Pose2d> robotPoseSupplier;
  private BooleanSupplier blueBoolSupplier;

  private double targetAngle = Math.PI;
  private Pose2d targetPose = new Pose2d();

  public TurretSubsystem(Supplier<Pose2d> robotPoseSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.blueBoolSupplier = () -> robotPoseSupplier.get().getX() < FieldConstants.kFieldCenterX;

    m_turretMotor.configure(
        Configs.Turret.turretMotorConfig19,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_turretMotor2.configure(
        Configs.Turret.turretMotorConfig21,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public double getTargetAngleRadians() {
    return targetAngle;
  }

  public Pose2d getTarget() {
    return targetPose;
  }

  public Command setVoltage(double volts) {
    return run(
        () -> {
          m_turretMotor.setVoltage(6 * volts);
        });
  }

  public void faceTarget(Supplier<Pose2d> targetPoseSupplier) {
    var cur_velocity = this.setpoint.velocity;
    targetPose = targetPoseSupplier.get();
    double robotToTargetRadians =
        Math.atan2(
            targetPose.getY() - robotPoseSupplier.get().getY(),
            targetPose.getX() - robotPoseSupplier.get().getX());
    this.goal =
        new TrapezoidProfile.State(
            robotToTargetRadians - robotPoseSupplier.get().getRotation().getRadians(), 0);
    this.setpoint = profile.calculate(0.020, this.setpoint, this.goal);
    ff = feedforward.calculateWithVelocities(cur_velocity, setpoint.velocity);
    fb = feedback.calculate(getPosRadians(), setpoint.position);

    double voltage = ff + fb;
    m_turretMotor.setVoltage(voltage);
  }

  public Command faceTargetCommand(Supplier<Pose2d> targetPoseSupplier) {
    return run(() -> faceTarget(targetPoseSupplier));
  }

  public Command faceHubCommand() {
    return faceTargetCommand(
        () ->
            blueBoolSupplier.getAsBoolean()
                ? FieldConstants.blueHubCenter
                : FieldConstants.redHubCenter);
  }

  public Command faceCenterLineCommand() {
    return faceTargetCommand(
        () ->
            new Pose2d(
                FieldConstants.kFieldCenterX, robotPoseSupplier.get().getY(), new Rotation2d()));
  }

  public Command faceNearestAllianceCommand() {
    return faceTargetCommand(
        () ->
            blueBoolSupplier.getAsBoolean()
                ? new Pose2d(0.0, robotPoseSupplier.get().getY(), new Rotation2d())
                : new Pose2d(
                    FieldConstants.kFieldMaxX, robotPoseSupplier.get().getY(), new Rotation2d()));
  }

  public double getVelocity() {
    return m_encoder19Teeth.getVelocity();
  }

  public double getAppliedVoltage() {
    return m_turretMotor.getAppliedOutput() * m_turretMotor.getBusVoltage();
  }

  public double getBusVoltage() {
    return m_turretMotor.getBusVoltage();
  }

  public double getCurrent() {
    return m_turretMotor.getOutputCurrent();
  }

  public double getSetpoint() {
    return setpoint.position;
  }

  public double getSetpointVelocity() {
    return setpoint.velocity;
  }

  public double getGoal() {
    return goal.position;
  }

  /**
   * This adjusts the turret position such that pi is facing forward and 0/2 pi is not in the range
   * of motion.
   */
  //   public double getPosRadians() {
  //     return (getTeethPosRadiansAdjusted() + Math.PI) % (2 * Math.PI);
  //   }

  /**
   * This adjusts to the more precise number of radians that isn't based on whole numbers of teeth.
   */
  private double getPosRadians() {
    double teeth19 = getEncoderTeeth19();
    double adjustment = (teeth19 - ((int) teeth19)) * (Math.PI / 100.0);
    return getTeethPosRadians() + adjustment;
  }

  /** Converts the number of teeth to radians */
  private double getTeethPosRadians() {
    return getCRTResult() * Math.PI / 100.0;
  }

  /**
   * The initial CRT result assumes that the large gear has 399 teeth (19 * 21), but it actually has
   * 200 teeth. We adjust for that here, with the assumption that 0 teeth is considered facing
   * forward in the center of its range of motion and that the turret has a range of motion smaller
   * than 360 degrees. This makes it the correct number of teeth.
   */
  //   private int get200TeethPos() {
  //     int result = getCRTInitResult();
  //     if (result > 200) {
  //       result = result - 199;
  //     }
  //     return result;
  //   }

  /**
   * We can use the Chinese Remainder Theorem (CRT) to find the position of the large gear given the
   * position of the 19-tooth and 21-tooth gears (this is possible because 19 and 21 are coprime,
   * meaning they do not share any factors). To do this, we set up the following equations (where
   * g19 is the position of the 19-tooth gear from 0 to 18 and g21 is the position of the 21-tooth
   * gear from 0 to 20):
   *
   * <p>x ≡ g19 (mod 19)
   *
   * <p>x ≡ g21 (mod 21)
   *
   * <p>To find x (which will be the number of teeth on the large gear), we use the CRT as follows:
   *
   * <p>M = m1 * m2 = 19 * 21 = 399
   *
   * <p>r1 = g19
   *
   * <p>r2 = g21
   *
   * <p>M1 = M / m1 = m2 = 21
   *
   * <p>M2 = M / m2 = m1 = 19
   *
   * <p>x1 = 10 (see below)
   *
   * <p>M1 * x1 ≡ 1 (mod m1)
   *
   * <p>21 * x1 ≡ 1 (mod 19) ≡ 210 (mod 19)
   *
   * <p>x2 = 10 (see below)
   *
   * <p>M2 * x2 ≡ 1 (mod m2)
   *
   * <p>19 * x2 ≡ 1 (mod 21) ≡ 190 (mod 21)
   *
   * <p>Therefore, we can combine these to conclude that:
   *
   * <p>x = (r1 * M1 * x1 + r2 * M2 * x2) mod 399
   *
   * <p>x = (g19 * 21 * 10 + g21 * 19 * 10) mod 399
   *
   * <p>This is the resulting equation used below: x = (210 * g19 + 190 * g21) mod 399
   */
  private int getCRTResult() {
    // Allows us to set 0 as forward and then makes this 180 degrees
    int teeth19 = (((int) getEncoderTeeth19()) + 100) % 19;
    int teeth21 = (((int) getEncoderTeeth21()) + 100) % 21;

    return (210 * teeth19 + 190 * teeth21) % 399;
  }

  private double getEncoderTeeth19() {
    return m_encoder19Teeth.getPosition();
  }

  private double getEncoderTeeth21() {
    return m_encoder21Teeth.getPosition();
  }
}
