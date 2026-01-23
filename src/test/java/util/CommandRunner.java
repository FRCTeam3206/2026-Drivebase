package util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Lets you run a command from a test where the robot isn't actually initialized
 */
public class CommandRunner {
    private CommandScheduler scheduler = CommandScheduler.getInstance();
    protected short schedulerTime = 200;
    
    public void start() {
        scheduler.enable();
    }

    public void stop() {
        scheduler.unregisterAllSubsystems();
        scheduler.cancelAll();
        scheduler.disable();
    }

    public void runOnce(Command command, Subsystem subsystem) throws InterruptedException {
        //best way i can figure out how to actually end the command is to add a ridiculously long timeout? Idk but it works
        Command actualCommand = command.ignoringDisable(true).withTimeout(Math.pow(2, 16));
        actualCommand.end(false);
        scheduler.schedule(actualCommand);
        while (!actualCommand.isFinished()) {
            scheduler.run();
            Thread.sleep(200);
        }
        return;
    }
}
