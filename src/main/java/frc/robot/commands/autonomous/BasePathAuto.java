package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

import java.util.Map;

public class BasePathAuto extends RamseteAutoBuilder {

    private static final RootNamespace root = new RootNamespace("base path auto");

    protected final Drivetrain drivetrain;

    public BasePathAuto(Drivetrain drivetrain, Map<String, Command> eventMap) {
        super(drivetrain::getPose2d, drivetrain::resetOdometry, drivetrain.getRamseteController(),
                drivetrain.getKinematics(),
                (leftMS, rightMS) -> drivetrain.pidSet(UnifiedControlMode.VELOCITY, leftMS, rightMS, drivetrain.getLeftPIDSettings(),
                        drivetrain.getRightPIDSettings(), drivetrain.getFeedForwardSettings()),
                eventMap, true, drivetrain);
        this.drivetrain = drivetrain;
    }

    private static SimpleMotorFeedforward getFeedForward(FeedForwardSettings feedForwardSettings) {
        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(feedForwardSettings.getkS(), feedForwardSettings.getkV(),
                feedForwardSettings.getkA());
        root.putNumber("ks", feedforward.ks);
        root.putNumber("kV", feedforward.kv);
        root.putNumber("kA", feedforward.ka);
        return feedforward;

    }

    private static PIDConstants getPID(PIDSettings pidSettings) {
        PIDConstants pid = new PIDConstants(pidSettings.getkP(), pidSettings.getkI(), pidSettings.getkD());
        root.putNumber("kp", pid.kP);
        root.putNumber("ki", pid.kI);
        root.putNumber("kd", pid.kD);
        return pid;
    }
}
