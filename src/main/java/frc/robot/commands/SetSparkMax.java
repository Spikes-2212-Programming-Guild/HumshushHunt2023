package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetSparkMax extends CommandBase {

    private CANSparkMax sparkMax;
    private double speed;

    public SetSparkMax(CANSparkMax sparkMax, double speed) {
        this.sparkMax = sparkMax;
        this.speed = speed;
    }

    @Override
    public void execute() {
        sparkMax.set(speed);
    }

    @Override
    public void end(boolean interrupted) {
        sparkMax.set(0);
    }
}
