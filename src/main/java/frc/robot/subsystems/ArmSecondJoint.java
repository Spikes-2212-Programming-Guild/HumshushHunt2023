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

import java.beans.Encoder;
import java.util.function.Supplier;

public class ArmSecondJoint extends SparkMaxGenericSubsystem {

    public static final int DISTANCE_PER_PULSE = -1;

    public static final int SECONDS_IN_MINUTE = 60;

    private static ArmSecondJoint instance;

    private final RelativeEncoder encoder;

    private final Namespace PIDNamespace = namespace.addChild("pid");
    private final Supplier<Double> kP = PIDNamespace.addConstantDouble("kP", 0);
    private final Supplier<Double> kI = PIDNamespace.addConstantDouble("kI", 0);
    private final Supplier<Double> kD = PIDNamespace.addConstantDouble("kD", 0);
    private final Supplier<Double> waitTime = PIDNamespace.addConstantDouble("wait time", 0);
    private final Supplier<Double> tolerance = PIDNamespace.addConstantDouble("tolerance", 0);
    private final PIDSettings PIDSettings;

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

    public static ArmSecondJoint getInstance() {
        if (instance == null) {
            instance = new ArmSecondJoint(
                    "arm second joint",
                    new CANSparkMax(RobotMap.CAN.ARM_SECOND_JOINT_SPARKMAX_1, CANSparkMaxLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.ARM_SECOND_JOINT_SPARKMAX_2, CANSparkMaxLowLevel.MotorType.kBrushless)
            );
            return instance;
        }
        return instance;
    }

    private ArmSecondJoint(String namespaceName, CANSparkMax master, CANSparkMax slave) {
        super(namespaceName, master, slave);
        this.encoder = master.getEncoder();
        encoder.setPositionConversionFactor(DISTANCE_PER_PULSE);
        slave.follow(master, false);
        this.PIDSettings = new PIDSettings(kP, kI, kD, waitTime, tolerance);
        this.feedForwardSettings = new FeedForwardSettings(kS, kV, kA, kG);
        this.trapezoidProfileSettings = new TrapezoidProfileSettings(trapezoidVelocity, trapezoidAcceleration);
        configureDashboard();
    }

    @Override
    public void configureLoop(PIDSettings PIDSettings, FeedForwardSettings feedForwardSettings,
                              TrapezoidProfileSettings trapezoidProfileSettings) {
        super.configureLoop(PIDSettings, feedForwardSettings, trapezoidProfileSettings);
        this.setConversionRates();
    }


    public void setConversionRates() {
        encoder.setPositionConversionFactor(DISTANCE_PER_PULSE);
        encoder.setVelocityConversionFactor(DISTANCE_PER_PULSE / SECONDS_IN_MINUTE);
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
        namespace.putNumber("encoder position", this::getEncoderPosition);
    }
}
