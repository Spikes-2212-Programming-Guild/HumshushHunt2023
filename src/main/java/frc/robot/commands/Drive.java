package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase {

    private final Drivetrain drivetrain;
    private final double leftSpeed;
    private final double rightSpeed;

    public Drive(Drivetrain drivetrain, double leftSpeed, double rightSpeed) {
        this.drivetrain = drivetrain;
        this.leftSpeed = leftSpeed;
        this.rightSpeed = rightSpeed;
    }

    @Override
    public void execute() {
        drivetrain.setSpeeds(leftSpeed, rightSpeed);
    }
}
