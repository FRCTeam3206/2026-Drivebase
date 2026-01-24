import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.DriveSubsystem;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;
import util.CommandRunner;

public class DriveSubsystemTest {
  private static final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private static final CommandRunner commands = new CommandRunner();

  @BeforeAll
  public static final void startup() throws InterruptedException {
    commands.start();
  }

  @AfterEach
  public final void resetRobot() {
    driveSubsystem.resetOdometry(new Pose2d());
  }

  @AfterAll
  public static final void done() {
    commands.stop();
  }

  @Test
  @Timeout(6)
  public final void drive() throws InterruptedException {
    final double initialX = driveSubsystem.getPose().getX();
    final double initialY = driveSubsystem.getPose().getY();
    commands.runOnce(
        driveSubsystem
            .driveCommand(() -> 5, () -> 5, () -> 0, () -> false)
            .withTimeout(4)
            .andThen(
                () -> {
                  double newX = driveSubsystem.getPose().getX();
                  double newY = driveSubsystem.getPose().getY();
                  // method overloading PMO!!!
                  System.out.println(
                      "Started at "
                          + initialX
                          + ','
                          + initialY
                          + '\n'
                          + "Ended at "
                          + newX
                          + ','
                          + newY
                          + '\n');
                  Assertions.assertTrue(newX > initialX, "Moved forward on the X coordinate");
                  Assertions.assertTrue(newY > initialY, "Moved forward on the Y coordinate");
                }),
        driveSubsystem);
    return;
  }

  @Test
  @Timeout(3)
  public final void turn() throws InterruptedException {
    final double initialRotation = driveSubsystem.getPose().getRotation().getDegrees();
    commands.runOnce(
        driveSubsystem
            .driveCommand(() -> 0, () -> 0, () -> 2, () -> false)
            .withTimeout(2)
            .andThen(
                () -> {
                  final double newRotation = driveSubsystem.getPose().getRotation().getDegrees();
                  System.out.println(
                      "Started at " + initialRotation + '\n' + "Ended at " + newRotation);
                  Assertions.assertTrue(newRotation > initialRotation, "Robot rotates");
                }),
        driveSubsystem);
    return;
  }
}
