import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;
import org.junit.jupiter.api.Assertions;

import frc.robot.subsystems.DriveSubsystem;

@Tag("real life")
public class DriveSubsystemTest {
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  @BeforeEach
  void setup() {}

  @Test
  @Timeout(6)
  void drive() throws InterruptedException {
    final double initialX = driveSubsystem.getPose().getX();
    final double initialY = driveSubsystem.getPose().getY();
    driveSubsystem.driveCommand(()->0.1, ()-> 0.1,()-> 0, ()->false).withTimeout(4).andThen(()->{
      Assertions.fail();
      Assertions.assertTrue(driveSubsystem.getPose().getX() < initialX);
      Assertions.assertTrue(driveSubsystem.getPose().getY() < initialY);
    });
    Thread.sleep(5000);
  }
}
