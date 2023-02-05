package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class lol extends SequentialCommandGroup {

    private Drivetrain drivetrain = Drivetrain.getInstance();

    public lol() {
        addRequirements(drivetrain);
        addCommands(
                new Drive(drivetrain, 0.2, 0.2).withTimeout(2),
                new Drive(drivetrain, 0, 0.1).withTimeout(1),
                new Drive(drivetrain, 0.2, 0.2).withTimeout(2),
                new Drive(drivetrain, 0, 0)
        );
    }
}
