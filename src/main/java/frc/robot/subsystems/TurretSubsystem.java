package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
    private SparkMax turretMotor = new SparkMax(0, MotorType.kBrushless);

    public TurretSubsystem() {}
}
