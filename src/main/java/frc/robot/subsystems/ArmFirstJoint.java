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

public class ArmFirstJoint extends SparkMaxGenericSubsystem {

    private static ArmFirstJoint instance;

    public static final int DISTANCE_PER_PULSE = -1;

    public static final int SECONDS_IN_MINUTE = 60;

    private final RelativeEncoder encoder; // @todo check encoder type

    private final Namespace pidNamespace = namespace.addChild("pid");
    private final Supplier<Double> kP = pidNamespace.addConstantDouble("kP", 0);
    private final Supplier<Double> kI = pidNamespace.addConstantDouble("kI", 0);
    private final Supplier<Double> kD = pidNamespace.addConstantDouble("kD", 0);
    private final Supplier<Double> waitTime = pidNamespace.addConstantDouble("wait time", 0);
    private final Supplier<Double> tolerance = pidNamespace.addConstantDouble("tolerance", 0);
    private final PIDSettings pidSettings;

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

    public static ArmFirstJoint getInstance() {
        if (instance == null) {
            instance = new ArmFirstJoint("arm first joint",
                    new CANSparkMax(RobotMap.CAN.ARM_FIRST_JOINT_SPARKMAX_MASTER,
                            CANSparkMaxLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.ARM_FIRST_JOINT_SPARKMAX_SLAVE_1,
                            CANSparkMaxLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.ARM_FIRST_JOINT_SPARKMAX_SLAVE_2,
                            CANSparkMaxLowLevel.MotorType.kBrushless)
            );
        }
        return instance;
    }

    private ArmFirstJoint(String namespaceName, CANSparkMax master, CANSparkMax slave1, CANSparkMax slave2) {
        super(namespaceName, master, slave1, slave2);
        this.encoder = master.getAlternateEncoder(RobotMap.CAN.ARM_FIRST_JOINT_SPARKMAX_SLAVE_1);
        setConversionFactors();
        slave2.follow(master, true);
        this.pidSettings = new PIDSettings(kP, kI, kD, waitTime, tolerance);
        this.feedForwardSettings = new FeedForwardSettings(kS, kV, kA, kG);
        this.trapezoidProfileSettings = new TrapezoidProfileSettings(trapezoidVelocity, trapezoidAcceleration);
        configureDashboard();
    }

    @Override
    public void configureLoop(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings,
                              TrapezoidProfileSettings trapezoidProfileSettings) {
        super.configureLoop(pidSettings, feedForwardSettings, trapezoidProfileSettings);
        this.setConversionFactors();
    }

    public void setConversionFactors() {
        encoder.setPositionConversionFactor(DISTANCE_PER_PULSE);
        encoder.setVelocityConversionFactor(DISTANCE_PER_PULSE / SECONDS_IN_MINUTE);
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public PIDSettings getPIDSettings() {
        return this.pidSettings;
    }

    public FeedForwardSettings getFeedForwardSettings() {
        return this.feedForwardSettings;
    }

    public TrapezoidProfileSettings getTrapezoidProfileSettings() {
        return this.trapezoidProfileSettings;
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("encoder position", this::getPosition);
    }
}
