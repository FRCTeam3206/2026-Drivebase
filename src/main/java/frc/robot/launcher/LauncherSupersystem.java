package frc.robot.launcher;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class LauncherSupersystem {
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    private final TransportSubsystem m_transport = new TransportSubsystem();
    private final AngleShooterSubsystem m_angleShooter = new AngleShooterSubsystem();
    private final TurretSubsystem m_turret;

    public LauncherSupersystem(Supplier<Pose2d> robotPoseSupplier) {
        m_turret = new TurretSubsystem(robotPoseSupplier);
    }

    public Command faceTargetCommand() {
        return m_turret.faceTargetCommand(null);
    }
}
