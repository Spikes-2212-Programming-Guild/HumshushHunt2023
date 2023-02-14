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

public class MoveSecondJoint extends CommandBase {

    private final RootNamespace rootNamespace = new RootNamespace("move second joint namespace");
    private final Supplier<Double> duration = rootNamespace.addConstantDouble("duration", 1);

    private MoveSmartMotorControllerGenericSubsystem move;
    private double startTime;

    private final ArmSecondJoint secondJoint;
    private final Supplier<Double> target;

    public MoveSecondJoint(ArmSecondJoint secondJoint, Supplier<Double> target) {
        this.secondJoint = secondJoint;
        this.target = target;
        addRequirements(secondJoint);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        double startPosition = secondJoint.getAbsolutePosition();
        move = new MoveSmartMotorControllerGenericSubsystem(secondJoint, secondJoint.getPIDSettings(),
                secondJoint.getFeedForwardSettings(), UnifiedControlMode.POSITION,
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
