package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkMaxGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.dashboard.Namespace;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.RobotMap;

import java.util.function.Supplier;

public class ArmFirstJoint extends SparkMaxGenericSubsystem {

    public static final double DISTANCE_PER_PULSE = -1;

    public static final int SECONDS_IN_MINUTE = 60;

    private static ArmFirstJoint instance;

    private final RelativeEncoder sparkMaxEncoder;
    private final DutyCycleEncoder absoluteEncoder;

    public final Supplier<Double> forwardSpeed = namespace.addConstantDouble("first joint forward speed", 0.1);
    public final Supplier<Double> backwardsSpeed = namespace.addConstantDouble("first joint backwards speed", -0.1);

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

    private ArmFirstJoint(String namespaceName, CANSparkMax master, CANSparkMax slave1) {
        super(namespaceName, master, slave1);
        sparkMaxEncoder = master.getAlternateEncoder(RobotMap.CAN.ARM_FIRST_JOINT_SPARKMAX_SLAVE);
        absoluteEncoder = new DutyCycleEncoder(RobotMap.DIO.ARM_FIRST_JOINT_ABSOLUTE_ENCODER);
        setConversionFactors();
        slave1.follow(master, true);
        pidSettings = new PIDSettings(kP, kI, kD, waitTime, tolerance);
        feedForwardSettings = new FeedForwardSettings(kS, kV, kA, kG);
        trapezoidProfileSettings = new TrapezoidProfileSettings(trapezoidVelocity, trapezoidAcceleration);
        configureDashboard();
    }

    public static ArmFirstJoint getInstance() {
        if (instance == null) {
            instance = new ArmFirstJoint("arm first joint",
                    new CANSparkMax(RobotMap.CAN.ARM_FIRST_JOINT_SPARKMAX_MASTER,
                            CANSparkMaxLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.ARM_FIRST_JOINT_SPARKMAX_SLAVE,
                            CANSparkMaxLowLevel.MotorType.kBrushless)
            );
        }
        return instance;
    }

    @Override
    public void configureLoop(PIDSettings PIDSettings, FeedForwardSettings feedForwardSettings,
                              TrapezoidProfileSettings trapezoidProfileSettings) {
        super.configureLoop(PIDSettings, feedForwardSettings, trapezoidProfileSettings);
        setConversionFactors();
    }

    public void setConversionFactors() {
        sparkMaxEncoder.setPositionConversionFactor(DISTANCE_PER_PULSE);
        sparkMaxEncoder.setVelocityConversionFactor(DISTANCE_PER_PULSE / SECONDS_IN_MINUTE);
        absoluteEncoder.setDistancePerRotation(DISTANCE_PER_PULSE);
        sparkMaxEncoder.setPosition(absoluteEncoder.getDistance());
    }

    public double getPosition() {
        return sparkMaxEncoder.getPosition();
    }

    public double getAbsolutePosition() {
        return absoluteEncoder.getDistance();
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
        namespace.putNumber("absolute encoder position", this::getAbsolutePosition);
        namespace.putNumber("spark max encoder position", this::getPosition);
    }
}
