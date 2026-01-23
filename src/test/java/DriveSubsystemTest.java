import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Timeout;

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
    commands.runOnce(
      driveSubsystem.driveCommand(()->5, ()->5, ()->0, ()->false).withTimeout(4).andThen(()->{
        double newX = driveSubsystem.getPose().getX();
        double newY = driveSubsystem.getPose().getY();
        System.out.println("okkkk");
        //method overloading PMO!!!
        System.out.println("" + initialX + '\n' + initialY + '\n' +newX +  '\n' + newY + '\n');
        Assertions.assertTrue(newX > initialX, "Moved forward on the X coordinate");
        Assertions.assertTrue(newY >  initialY, "Moved forward on the Y coordinate");
      }),
      driveSubsystem
    );
    return;
    }
}
