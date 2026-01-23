package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransportConstants;

public class TransportSubsystem extends SubsystemBase {
    private final SparkMax m_TransportMotor = new SparkMax(TransportConstants.kTransportMotorCANid, MotorType.kBrushless);
    public TransportSubsystem() {
    }
    public Command ballTransport() {
        return this.run(() -> m_TransportMotor.set(0.8))
            .withTimeout(1)
            .finallyDo(() -> m_TransportMotor.set(0));
    }
}
