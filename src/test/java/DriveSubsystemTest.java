import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

@Tag("real life")
public class DriveSubsystemTest {
  // private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  @BeforeEach
  void setup() {}

  @Test
  void drive() {
    final double initialX = 0;
    final double initialY = 0;
    /* idk how to test with async commands lol,,,
    driveSubsystem
        .runOnce(
            () -> {
              driveSubsystem.drive(0.1, 0.1, 0, false);
            })
        .withTimeout(5)
        .andThen(
            () -> {
                driveSubsystem.drive(0, 0, 0, false);
                System.out.println(driveSubsystem.getPose().getX());
                System.out.println(driveSubsystem.getPose().getY());
                Assertions.assertTrue(driveSubsystem.getPose().getX() > initialX);
                Assertions.assertTrue(driveSubsystem.getPose().getY() > initialY);
            });
     */
  }
}
