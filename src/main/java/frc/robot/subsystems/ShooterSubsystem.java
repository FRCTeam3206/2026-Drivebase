package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax m_topWheel = new SparkMax(ShooterConstants.kTopLauncherMotor, null);
    private final SparkMax m_bottomWheel = new SparkMax(ShooterConstants.kBottomLauncherMotor, null);

    public Command launchCommand() {
    return this.run(() -> m_topWheel.set(0.8))
            .andThen(() -> m_bottomWheel.set(0.8))
            .finallyDo(() -> {m_topWheel.stopMotor(); m_bottomWheel.stopMotor();});
}
  }
 

