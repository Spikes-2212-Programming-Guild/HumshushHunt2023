package frc.robot.commands;

import com.spikes2212.command.drivetrains.commands.DriveTankWithPID;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;

public class Climb extends CommandBase {

    private static final RootNamespace namespace = new RootNamespace("climb");
    private static final Supplier<Double> preClimbSpeed = namespace.addConstantDouble("pre climb speed", 0.6);
    private static final Supplier<Double> midClimbSpeed = namespace.addConstantDouble("mid speed climb", 0.3);
    private static final Supplier<Double> preClimbTolerance = namespace.addConstantDouble("pre climb tolerance", 15);
    private static final Supplier<Double> midClimbTolerance = namespace.addConstantDouble("mid climb tolerance", 3);
    private static final Supplier<Double> yawSetpoint = namespace.addConstantDouble("yaw setpoint", 75);
    private static final Supplier<Double> waitTime = namespace.addConstantDouble("wait time", 1.5);

    private final Drivetrain drivetrain;

    private boolean startedClimbing;
    private double lastTimeNotOnTarget;

    public Climb(Drivetrain drivetrain) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        startedClimbing = false;
        lastTimeNotOnTarget = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        double pitch = drivetrain.getPitch();
        if (!startedClimbing) {
            drivetrain.arcadeDrive(preClimbSpeed.get(), 0);
            startedClimbing = Math.abs(pitch) > preClimbTolerance.get();
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        } else {
            if (Math.abs(pitch) <= midClimbTolerance.get()) {
                if (Math.abs(drivetrain.getYaw()) <= yawSetpoint.get()) {
                    drivetrain.arcadeDrive(0, midClimbSpeed.get());
                } else {
                    drivetrain.arcadeDrive(Math.signum(pitch) * midClimbSpeed.get(), 0);
                    lastTimeNotOnTarget = Timer.getFPGATimestamp();
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return lastTimeNotOnTarget >= waitTime.get();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
