package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.services.ArmGravityCompensation;
import frc.robot.subsystems.ArmFirstJoint;
import frc.robot.subsystems.ArmSecondJoint;

import java.util.function.Supplier;

public class MoveFirstJoint extends CommandBase {

    private final RootNamespace rootNamespace = new RootNamespace("move first joint namespace");
    private final Supplier<Double> duration = rootNamespace.addConstantDouble("duration", 1);

    private MoveSmartMotorControllerGenericSubsystem move;
    private double startTime;

    private final ArmFirstJoint firstJoint;
    private final Supplier<Double> target;

    public MoveFirstJoint(ArmFirstJoint firstJoint, Supplier<Double> target) {
        this.firstJoint = firstJoint;
        this.target = target;
        addRequirements(firstJoint);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        double startPosition = firstJoint.getAbsolutePosition();
        move = new MoveSmartMotorControllerGenericSubsystem(firstJoint, firstJoint.getPIDSettings(),
                firstJoint.getFeedForwardSettings(), UnifiedControlMode.POSITION,
                () -> startPosition + ((target.get() - startPosition) / duration.get())
                        * Math.min((Timer.getFPGATimestamp() - startTime), duration.get()));
    }


    @Override
    public void execute() {
        move.execute();
    }

    @Override
    public void end(boolean interrupted) {
        move.end(interrupted);
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}
