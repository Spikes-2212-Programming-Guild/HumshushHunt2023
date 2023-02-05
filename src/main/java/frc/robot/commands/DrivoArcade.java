package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DrivoArcade extends CommandBase {

    private final Drivetrain drivetrain;
    private double move;
    private double rotate;

    public DrivoArcade(Drivetrain drivetrain, double move, double rotate) {
        this.drivetrain = drivetrain;
        this.move = move;
        this.rotate = rotate;
    }

    @Override
    public void execute() {
        move = MathUtil.applyDeadband(move, 0.05);
        rotate = MathUtil.applyDeadband(rotate, 0.05);
        double leftSpeed = move - rotate;
        double rightSpeed = move + rotate;
        drivetrain.setSpeeds(leftSpeed, rightSpeed);
    }
}
