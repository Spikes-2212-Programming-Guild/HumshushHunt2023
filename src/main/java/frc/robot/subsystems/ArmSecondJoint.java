package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkMaxGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.dashboard.Namespace;
import frc.robot.RobotMap;

import java.util.function.Supplier;

public class ArmSecondJoint extends SparkMaxGenericSubsystem {

    public static final int DISTANCE_PER_PULSE = -1;

    private static ArmSecondJoint instance;

    private final RelativeEncoder encoder;

    private final Namespace PIDNamespace = namespace.addChild("pid");
    private final Supplier<Double> kP = PIDNamespace.addConstantDouble("kP", 0);
    private final Supplier<Double> kI = PIDNamespace.addConstantDouble("kI", 0);
    private final Supplier<Double> kD = PIDNamespace.addConstantDouble("kD", 0);
    private final Supplier<Double> waitTime = PIDNamespace.addConstantDouble("wait time", 0);
    private final Supplier<Double> tolerance = PIDNamespace.addConstantDouble("tolerance", 0);
    private final com.spikes2212.control.PIDSettings PIDSettings;

    private final Namespace feedForwardNamespace = namespace.addChild("feed forward");
    private final Supplier<Double> kS = feedForwardNamespace.addConstantDouble("kS", 0);
    private final Supplier<Double> kV = feedForwardNamespace.addConstantDouble("kV", 0);
    private final Supplier<Double> kA = feedForwardNamespace.addConstantDouble("kA", 0);
    private final Supplier<Double> kG = feedForwardNamespace.addConstantDouble("kG", 0);
    private final FeedForwardSettings feedForwardSettings;

    private final Namespace trapezoidSettingsNamespace = namespace.addChild("trapezoid profile settings");
    private final Supplier<Double> trapezoidVelocity = trapezoidSettingsNamespace.addConstantDouble("velocity", 0);
    private final Supplier<Double> trapezoidAcceleration = trapezoidSettingsNamespace.addConstantDouble
            ("acceleration", 0);
    private final TrapezoidProfileSettings trapezoidProfileSettings;

    private ArmSecondJoint(String namespaceName, CANSparkMax sparkMax) {
        super(namespaceName, sparkMax);
        this.encoder = sparkMax.getAlternateEncoder(DISTANCE_PER_PULSE);
        this.PIDSettings = new PIDSettings(kP, kI, kD, waitTime, tolerance);
        this.feedForwardSettings = new FeedForwardSettings(kS, kV, kA, kG);
        this.trapezoidProfileSettings = new TrapezoidProfileSettings(trapezoidVelocity, trapezoidAcceleration);
    }

    public static ArmSecondJoint getInstance() {
        if (instance == null) {
            instance = new ArmSecondJoint(
                    "arm second joint",
                    new CANSparkMax(RobotMap.CAN.ARM_SECOND_JOINT_SPARKMAX_1, CANSparkMaxLowLevel.MotorType.kBrushless)
            );
            return instance;
        }
        return instance;
    }

    public PIDSettings getPIDSettings() {
        return this.PIDSettings;
    }

    public FeedForwardSettings getFeedForwardSettings() {
        return this.feedForwardSettings;
    }

    public TrapezoidProfileSettings getTrapezoidProfileSettings() {
        return this.trapezoidProfileSettings;
    }

    public double getEncoderPosition() {
        return encoder.getPosition();
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("second joint encoder position", this::getEncoderPosition);
    }
}
