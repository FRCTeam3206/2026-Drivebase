package util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Lets you run a command from a test where the robot isn't actually initialized
 */
public class CommandRunner {
    private CommandScheduler scheduler = CommandScheduler.getInstance();
    protected short schedulerTime = 2000;
    private boolean running = true;
    
    public void start() throws InterruptedException {
        scheduler.enable();
        this.running = true;
        while (running) {
            scheduler.run();
            try {
                Thread.sleep(schedulerTime);
            } catch (Throwable error) {
                System.err.println(error);
            }
        }
    }

    public void stop() {
        this.running = false;
        scheduler.disable();
    }

    public void addSubsystem(SubsystemBase subsystem) {
        scheduler.registerSubsystem(subsystem);
    }

    public void runOnce(Command command) {
        System.out.println(":3 :3 :3");
        scheduler.schedule(command.ignoringDisable(true));
    }
}
