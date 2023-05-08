package frc.robot.commands;

import com.spikes2212.command.drivetrains.commands.DriveArcade;
import frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;

public class TurnInAngle extends DriveArcade {
    private final Drivetrain drivetrain;
    private final double angle;
    private double wantedAngle;

    public TurnInAngle(Drivetrain drivetrain, double angle, boolean toTheLeft) {
        super(drivetrain, () -> 0.0, getRotateSpeed(toTheLeft));
        this.drivetrain = drivetrain;
        this.angle = angle;
    }

    private static Supplier<Double> getRotateSpeed(boolean toTheLeft) {
        if (toTheLeft)
            return () -> -Drivetrain.ROTATE_SPEED.get();
        return Drivetrain.ROTATE_SPEED;
    }

    @Override
    public void initialize() {
        drivetrain.resetGyro();
        double currentAngle = drivetrain.getYaw();
        wantedAngle = currentAngle + angle;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(wantedAngle - drivetrain.getYaw()) < Drivetrain.ROTATE_TOLERANCE.get();
    }
}