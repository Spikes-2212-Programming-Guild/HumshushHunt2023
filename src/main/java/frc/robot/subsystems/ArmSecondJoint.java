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

public class ArmSecondJoint extends SparkMaxGenericSubsystem {

    public static final double DISTANCE_PER_PULSE = -1;

    public static final int SECONDS_IN_MINUTE = 60;

    private static ArmSecondJoint instance;

    private final DutyCycleEncoder absoluteEncoder;
    private final RelativeEncoder sparkMaxEncoder;

    public final Supplier<Double> forwardSpeed = namespace.addConstantDouble("second joint forward speed", 0.1);
    public final Supplier<Double> backwardsSpeed = namespace.addConstantDouble("second joint backwards speed", -0.1);

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

    public static ArmSecondJoint getInstance() {
        if (instance == null) {
            instance = new ArmSecondJoint(
                    "arm second joint",
                    new CANSparkMax(RobotMap.CAN.ARM_SECOND_JOINT_SPARKMAX_MASTER,
                            CANSparkMaxLowLevel.MotorType.kBrushless));
            return instance;
        }
        return instance;
    }

    private ArmSecondJoint(String namespaceName, CANSparkMax master) {
        super(namespaceName, master);
        sparkMaxEncoder = master.getEncoder();
        absoluteEncoder = new DutyCycleEncoder(RobotMap.DIO.ARM_SECOND_JOINT_ABSOLUTE_ENCODER);
        configureEncoders();
        pidSettings = new PIDSettings(kP, kI, kD, waitTime, tolerance);
        feedForwardSettings = new FeedForwardSettings(kS, kV, kA, kG);
        trapezoidProfileSettings = new TrapezoidProfileSettings(trapezoidVelocity, trapezoidAcceleration);
        configureDashboard();
    }

    @Override
    public void configureLoop(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings,
                              TrapezoidProfileSettings trapezoidProfileSettings) {
        super.configureLoop(pidSettings, feedForwardSettings, trapezoidProfileSettings);
        configureEncoders();
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

    private void configureEncoders() {
        sparkMaxEncoder.setPositionConversionFactor(DISTANCE_PER_PULSE);
        sparkMaxEncoder.setVelocityConversionFactor(DISTANCE_PER_PULSE / SECONDS_IN_MINUTE);
        absoluteEncoder.setDistancePerRotation(DISTANCE_PER_PULSE);
        sparkMaxEncoder.setPosition(absoluteEncoder.getDistance());
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("absolute encoder position", this::getAbsolutePosition);
        namespace.putNumber("spark max encoder position", this::getPosition);
    }
}
