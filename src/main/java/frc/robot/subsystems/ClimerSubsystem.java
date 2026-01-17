package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimerSubsystem extends SubsystemBase {
  private SparkMax Motor = new SparkMax(0, MotorType.kBrushless);
  private SparkRelativeEncoderSim encoderclimb = new SparkRelativeEncoderSim(Motor);
  private SparkMaxSim makethesimulatorclimbermotorsimultator = new SparkMaxSim(Motor, null);
  private RelativeEncoder climbchecker;

  public ClimerSubsystem() {
    this.climbchecker = Motor.getEncoder();
  }

  public void RasieElevator() {
    this.Motor.set(0.25);
  }

  public void StopElevator() {
    this.Motor.stopMotor();
  }

  public void LowerElevator() {
    this.Motor.set(-0.25);
  }

  @Override
  public void periodic() {

    if (this.encoderclimb.getPosition() < 0.1 || this.encoderclimb.getPosition() > 1) {
      this.StopElevator();
    }
  }
}
