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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TurretConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@Logged
public class TurretSubsystem extends SubsystemBase {
  private final SparkMax m_turretMotor =
      new SparkMax(TurretConstants.kTurretCANId, MotorType.kBrushless);
  private final AbsoluteEncoder m_absEncoderMotor = m_turretMotor.getAbsoluteEncoder();

  private final DutyCycleEncoder m_dutyCycleEncoder =
      new DutyCycleEncoder(TurretConstants.kEncoderOnlyPort, 0.0, TurretConstants.kEncoderZero);
  // Unit conversion is taken care of elsewhere to ensure it doesn't interfere with setting zero.

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
        Configs.Turret.turretMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public double getTargetAngleRadians() {
    return targetAngle;
  }

  public Pose2d getTarget() {
    return targetPose;
  }

  public Command setVoltageCommand(DoubleSupplier voltsSupplier) {
    return run(
        () -> {
          m_turretMotor.setVoltage(6 * voltsSupplier.getAsDouble());
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
    return m_absEncoderMotor.getVelocity();
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
   * This adjusts to the more precise number of radians that isn't based on whole numbers of teeth.
   */
  private double getPosRadians() {
    double teethMotor = getEncoderTeethMotor();
    double adjustment = (teethMotor - ((int) teethMotor)) * (Math.PI / 100.0);
    return getTeethPosRadians() + adjustment;
  }

  /** Converts the number of teeth to radians */
  private double getTeethPosRadians() {
    return getCRTResult() * Math.PI / 100.0;
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
  private int getCRTResult() {
    int teethMotor = (int) getEncoderTeethMotor();
    int teethOther = (int) getEncoderTeethOther();

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
}
