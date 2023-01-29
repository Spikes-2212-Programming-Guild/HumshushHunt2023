package frc.robot.commands;

import com.spikes2212.command.drivetrains.commands.DriveTankWithPID;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;

public class Climb extends CommandBase {

    private static final RootNamespace namespace = new RootNamespace("climb");
    private static final Supplier<Double> speed = namespace.addConstantDouble("speed", 0);
    private static final Supplier<Double> tolerance = namespace.addConstantDouble("tolerance", 0);
    private static final Supplier<Double> setpoint = namespace.addConstantDouble("setpoint", 0);

    private final Drivetrain drivetrain;

    private boolean startedClimbing;

    public Climb(Drivetrain drivetrain) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        startedClimbing = false;
    }

    @Override
    public void execute() {
        double pitch = drivetrain.getPitch();
        if (!startedClimbing) {
            drivetrain.arcadeDrive(speed.get(), 0);
            startedClimbing = Math.abs(pitch) > tolerance.get();
        } else {
            if (startedClimbing) {
                if (Math.abs(pitch) <= tolerance.get()) {
                    climbOnCenter().schedule();
                } else {
                    if ((int) Math.signum(pitch) == 1) {
                        drivetrain.arcadeDrive(speed.get(), 0);
                    } else {
                        drivetrain.arcadeDrive(-speed.get(), 0);
                    }
                }
            }
        }
    }

    public DriveTankWithPID climbOnCenter() {
        return new DriveTankWithPID(drivetrain, drivetrain.getLeftPIDSettings(), drivetrain.getRightPIDSettings(),
                setpoint.get(), setpoint.get(),
                drivetrain::getLeftEncoderPosition, drivetrain::getRightEncoderPosition) {
            @Override
            public void initialize() {
                Drivetrain drivetrain1 = (Drivetrain) drivetrain;
                drivetrain1.resetEncoders();
            }
        };
    }
}
