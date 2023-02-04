package frc.robot.commands;

import com.spikes2212.command.drivetrains.commands.DriveArcade;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;

public class DriveAndStop extends SequentialCommandGroup {

    Drivetrain drivetrain = Drivetrain.getInstance();

    private double speed;
    private double rotate;

    public DriveAndStop(double speed, double rotate) {
        addRequirements(drivetrain);
        addCommands(drive(speed, rotate).withTimeout(3), new WaitCommand(3),
                drive(-speed, -rotate));
    }

    private DriveArcade drive(double speed, double rotate) {
        if (Math.abs(speed) > 0.05 || Math.abs(rotate) > 0.05)
            return new DriveArcade(drivetrain, speed, rotate);
        return new DriveArcade(drivetrain, 0, 0);
    }
}
