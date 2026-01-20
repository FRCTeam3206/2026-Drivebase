package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ClimberConstants;

@Logged
public class ClimberSubsystem extends SubsystemBase {
  private SparkMax climbMotor =
      new SparkMax(ClimberConstants.kClimberConstantCanId, MotorType.kBrushless);
  // private SparkRelativeEncoderSim encoderclimb = new SparkRelativeEncoderSim(motor);
  // private SparkMaxSim makethesimulatorclimbermotorsimultator = new SparkMaxSim(motor, null);
  private RelativeEncoder climbPostionEncoder;

  public ClimberSubsystem() {
    climbMotor.configure(
        Configs.ClimberModule.climberConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    this.climbPostionEncoder = climbMotor.getEncoder();
  }

  public void rasieElevator() {
    setSpeed(0.25);
  }

  public void stopElevator() {
    this.climbMotor.stopMotor();
  }

  public void lowerElevator() {
    setSpeed(-0.25);
  }

  public Command raiseCommand() {
    return run(this::rasieElevator).finallyDo(this::stopElevator);
  }

  public Command lowerCommand() {
    return run(this::lowerElevator).finallyDo(this::stopElevator);
  }

  public double getClimberPosition() {
    return this.climbPostionEncoder.getPosition();
  }

  public void setSpeed(double speed) {
    if ((getClimberPosition() < ClimberConstants.climbMax && speed > 0) || speed < 0) {
      climbMotor.set(speed);
    } else climbMotor.set(0);
    var backend = Epilogue.getConfig().backend;
    backend.log("Climb Speed", speed);
  }

  public Command zero() {
    return this.runOnce(
        () -> {
          climbMotor.getEncoder().setPosition(0);
        });
  }

  @Override
  public void periodic() {

    // if (this.encoderclimb.getPosition() < 0.1 || this.encoderclimb.getPosition() > 1) {
    //   this.stopElevator();
    // }

  }
}
