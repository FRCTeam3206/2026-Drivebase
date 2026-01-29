package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax m_TopIntakeMotor = new SparkMax(IntakeConstants.kTopIntakeMotor, MotorType.kBrushless);
  private final SparkMax m_BottomIntakeMotor = new SparkMax(IntakeConstants.kBottomIntakeMotor, MotorType.kBrushless);
  private final SparkMax m_DeployIntake = new SparkMax(IntakeConstants.kDeployIntakeMotor, MotorType.kBrushless);
  private final SparkMaxSim TopIntakeMotorSim = new SparkMaxSim(m_TopIntakeMotor, null);
  private final SparkMaxSim BottomIntakeMotorSim = new SparkMaxSim(m_BottomIntakeMotor, null);
  private final SparkMaxSim DeployIntakeSim = new SparkMaxSim(m_DeployIntake,  null);

  public IntakeSubsystem() {}

  public Command intakeBalls() {
    return this.run(
        () -> {
          m_TopIntakeMotor.set(IntakeConstants.kTopIntakeMotorSpeed);
          m_BottomIntakeMotor.set(IntakeConstants.kTopIntakeMotorSpeed);
          TopIntakeMotorSim.setVelocity(IntakeConstants.kBottomIntakeMotorSpeed);
          BottomIntakeMotorSim.setVelocity(IntakeConstants.kBottomIntakeMotorSpeed);
        });
  }

  public Command stopIntakeBalls() {
    return this.run(
        () -> {
          m_TopIntakeMotor.set(0);
          m_BottomIntakeMotor.set(0);
          TopIntakeMotorSim.setVelocity(0);
          BottomIntakeMotorSim.setVelocity(0);
        });
  }

  public Command deployIntake() {
    return this.run(() -> {m_DeployIntake.set(IntakeConstants.kDeployIntakeMotorSpeed); 
      DeployIntakeSim.setVelocity(IntakeConstants.kDeployIntakeMotorSpeed); })
        .withTimeout(0.2)
        .finallyDo(() -> {m_DeployIntake.set(0);
          DeployIntakeSim.setVelocity(0);});
  }

  public Command returnIntake() {
    return this.run(() -> {m_DeployIntake.set(-IntakeConstants.kDeployIntakeMotorSpeed); 
      DeployIntakeSim.setVelocity(-IntakeConstants.kDeployIntakeMotorSpeed);})
        .withTimeout(0.2)
        .finallyDo(() -> {m_DeployIntake.set(0); 
          DeployIntakeSim.setVelocity(0);});
  }
}
