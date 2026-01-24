package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax m_TopIntakeMotor =
      new SparkMax(IntakeConstants.kTopIntakeMotor, MotorType.kBrushless);
  private final SparkMax m_BottomIntakeMotor =
      new SparkMax(IntakeConstants.kBottomIntakeMotor, MotorType.kBrushless);
  private final SparkMax m_DeployIntake =
      new SparkMax(IntakeConstants.kDeployIntakeMotor, MotorType.kBrushless);

  public IntakeSubsystem() {}

  public Command intakeBalls() {
    return this.run(
        () -> {
          m_TopIntakeMotor.set(0.8);
          m_BottomIntakeMotor.set(0.8);
        });
  }

  public Command stopIntakeBalls() {
    return this.run(
        () -> {
          m_TopIntakeMotor.set(0);
          m_BottomIntakeMotor.set(0);
        });
  }

  public Command deployIntake() {
    return this.run(() -> m_DeployIntake.set(0.8))
        .withTimeout(0.2)
        .finallyDo(() -> m_DeployIntake.set(0));
  }

  public Command returnIntake() {
    return this.run(() -> m_DeployIntake.set(-0.8))
        .withTimeout(0.2)
        .finallyDo(() -> m_DeployIntake.set(0));
  }
}
