import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;
import org.junit.jupiter.api.Timeout;
import org.junit.jupiter.api.TestInstance.Lifecycle;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeAll;

import frc.robot.subsystems.DriveSubsystem;
import util.CommandRunner;

public class DriveSubsystemTest {
  private final static DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final static CommandRunner commands = new CommandRunner();

  @BeforeAll
  public static final void startup() throws InterruptedException {
    System.out.println("i have to piss readlly hard");
    commands.start();
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
    System.out.println("mrow");
    commands.addSubsystem(driveSubsystem);
    commands.runOnce(
      driveSubsystem.driveCommand(()->0.1, ()->0.1, ()->0, ()->false).withTimeout(4).andThen(()->{
        System.out.println("okkkk");
        Assertions.assertTrue(driveSubsystem.getPose().getX() > initialX);
        Assertions.assertTrue(driveSubsystem.getPose().getY() > initialY);
      })
    );
    }
}
