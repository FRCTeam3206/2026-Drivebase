package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;

@Logged
public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax topWheel =
      new SparkMax(ShooterConstants.kTopLauncherMotor, MotorType.kBrushless);
  private final SparkMax bottomWheel =
      new SparkMax(ShooterConstants.kBottomLauncherMotor, MotorType.kBrushless);
  private final SparkMaxSim topWheelSim = new SparkMaxSim(topWheel, null);
  private final SparkMaxSim bottomWheelSim = new SparkMaxSim(bottomWheel, null);
  private final RelativeEncoder topEncoder = topWheel.getEncoder();
  private final RelativeEncoder bottomEncoder = bottomWheel.getEncoder();
  private final SparkRelativeEncoderSim topEncoderSim = new SparkRelativeEncoderSim(topWheel);
  private final SparkRelativeEncoderSim bottomEncoderSim = new SparkRelativeEncoderSim(bottomWheel);

  public ShooterSubsystem() {
    topWheel.configure(
        Configs.Shooter.shooterConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    bottomWheel.configure(
        Configs.Shooter.shooterConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public Command launchCommand() {
    return this.run(
            () -> {
              topWheel.set(0.8);
              topWheelSim.setVelocity(0.8);
            })
        .andThen(
            () -> {
              bottomWheel.set(-0.8);
              bottomWheelSim.setVelocity(-0.8);
            })
        .finallyDo(
            () -> {
              topWheel.stopMotor();
              topWheelSim.setVelocity(0);
              bottomWheel.stopMotor();
              bottomWheelSim.setVelocity(0);
            });
  }
}
