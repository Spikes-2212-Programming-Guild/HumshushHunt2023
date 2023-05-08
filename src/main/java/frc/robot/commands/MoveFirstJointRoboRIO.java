package frc.robot.commands;

import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmFirstJoint;

import java.util.function.Supplier;

public class MoveFirstJointRoboRIO extends CommandBase {

    private final ArmFirstJoint firstJoint;
    private final PIDController pidController;
    private final Supplier<Double> target;
    private final Supplier<Double> moveDuration;
    private final Supplier<Double> waitTime;
    private Supplier<Double> setpoint;
    private double lastTimeNotOnTarget;

    public MoveFirstJointRoboRIO(ArmFirstJoint firstJoint, Supplier<Double> target, Supplier<Double> moveDuration,
                                 Supplier<Double> waitTime) {
        this.firstJoint = firstJoint;
        this.pidController = new PIDController(firstJoint.getPIDSettings().getkP(),
                firstJoint.getPIDSettings().getkI(), firstJoint.getPIDSettings().getkD());
        this.target = target;
        this.moveDuration = moveDuration;
        this.waitTime = waitTime;
    }

    @Override
    public void initialize() {
        double startTime = Timer.getFPGATimestamp();
        double startPosition = firstJoint.getAbsolutePosition();
        setpoint = () -> startPosition + ((target.get() - startPosition) / moveDuration.get())
                * Math.min((Timer.getFPGATimestamp() - startTime), moveDuration.get());
        lastTimeNotOnTarget = startTime;
        firstJoint.configureLoop(firstJoint.getPIDSettings(), firstJoint.getFeedForwardSettings());
    }

    @Override
    public void execute() {
        pidController.setPID(firstJoint.getPIDSettings().getkP(),
                firstJoint.getPIDSettings().getkI(), firstJoint.getPIDSettings().getkD());
        firstJoint.pidSet(UnifiedControlMode.PERCENT_OUTPUT, setpoint.get(), firstJoint.getPIDSettings(),
                firstJoint.getFeedForwardSettings());
    }

    @Override
    public boolean isFinished() {
        boolean onTarget =
                Math.abs(target.get() - firstJoint.getAbsolutePosition()) <= firstJoint.getPIDSettings().getTolerance();
        if (!onTarget) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }
        return Timer.getFPGATimestamp() - lastTimeNotOnTarget >= waitTime.get();
    }
}
