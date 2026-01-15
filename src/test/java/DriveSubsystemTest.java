import frc.robot.subsystems.DriveSubsystem;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

@Tag("real life")
public class DriveSubsystemTest {
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  @BeforeEach
  void setup() {}

  @Test
  void drive() {
    final double initialX = 0;
    final double initialY = 0;
    driveSubsystem
        .runOnce(
            () -> {
              driveSubsystem.drive(0.1, 0.1, 0, false);
            })
        .withTimeout(5)
        .andThen(
            () -> {
              driveSubsystem.drive(0, 0, 0, false);
              Assertions.assertTrue(driveSubsystem.getPose().getX() > initialX);
              Assertions.assertTrue(driveSubsystem.getPose().getY() > initialY);
            });
  }
}
