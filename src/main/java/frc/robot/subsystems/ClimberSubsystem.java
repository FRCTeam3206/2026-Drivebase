package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private SparkMax motor = new SparkMax(ClimberConstants.kClimberConstantCanId, MotorType.kBrushless);
  private SparkRelativeEncoderSim encoderclimb = new SparkRelativeEncoderSim(motor);
  private SparkMaxSim makethesimulatorclimbermotorsimultator = new SparkMaxSim(motor, null);
  private RelativeEncoder climbchecker;

  public ClimberSubsystem() {
    motor.configure(
        Configs.ClimberModule.climberConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    this.climbchecker = motor.getEncoder();
  }

  public void rasieElevator() {
    this.motor.set(0.25);
  }

  public void stopElevator() {
    this.motor.stopMotor();
  }

  public void lowerElevator() {
    this.motor.set(-0.25);
  }

  public Command raiseCommand () {
    return run(this::rasieElevator).finallyDo(this::stopElevator);
  }

  public Command lowerCommand () {
    return run(this::lowerElevator).finallyDo(this::stopElevator);
  }

  @Override
  public void periodic() {

    // if (this.encoderclimb.getPosition() < 0.1 || this.encoderclimb.getPosition() > 1) {
    //   this.stopElevator();
    // }


  }
}
