package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SparkMaxGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.dashboard.Namespace;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotMap;

import java.util.function.Supplier;

public class ArmFirstJoint extends SparkMaxGenericSubsystem {

    public static final double DEGREES_PER_ROTATION = 360;

    /**
     * Ofek said.
     */
    public static final double GEAR_RATIO_MOTOR_TO_ABSOLUTE_ENCODER = 1 / ((60.0 / 15) * (50.0 / 14) * (48 / 14.0) * (28 / 12.0));

    public static final int SECONDS_IN_MINUTE = 60;

    private static final double ABSOLUTE_ENCODER_OFFSET = 180;

    private static ArmFirstJoint instance;

    private final RelativeEncoder sparkMaxEncoder;
    private final DutyCycleEncoder absoluteEncoder;

    public final Supplier<Double> forwardSpeed = namespace.addConstantDouble("forward speed", 0.175);
    public final Supplier<Double> backwardsSpeed = namespace.addConstantDouble("backwards speed", -0.175);

    private final Namespace pidNamespace = namespace.addChild("pid");
    private final Supplier<Double> kP = pidNamespace.addConstantDouble("kP", 0.07);
    private final Supplier<Double> kI = pidNamespace.addConstantDouble("kI", 0);
    private final Supplier<Double> kD = pidNamespace.addConstantDouble("kD", 0);
    private final Supplier<Double> waitTime = pidNamespace.addConstantDouble("wait time", 0.5);
    private final Supplier<Double> tolerance = pidNamespace.addConstantDouble("tolerance", 1);
    private final PIDSettings pidSettings;

    private final Namespace feedForwardNamespace = namespace.addChild("feed forward");
    private final Supplier<Double> kS = feedForwardNamespace.addConstantDouble("kS", 0);
    private final Supplier<Double> kV = feedForwardNamespace.addConstantDouble("kV", 0);
    private final Supplier<Double> kA = feedForwardNamespace.addConstantDouble("kA", 0);
    private final Supplier<Double> kG = feedForwardNamespace.addConstantDouble("kG", -1.35);
    private final FeedForwardSettings feedForwardSettings;

    private final Namespace trapezoidProfileNamespace = namespace.addChild("trapezoid profile settings");
    private final Supplier<Double> maxVelocity = trapezoidProfileNamespace.addConstantDouble("max velocity", 0);
    private final Supplier<Double> trapezoidAcceleration = trapezoidProfileNamespace.addConstantDouble
            ("acceleration", 0);
    private final TrapezoidProfileSettings trapezoidProfileSettings;

    private final Namespace calibrations = namespace.addChild("calibrations");
    public final Supplier<Double> lm1 = calibrations.addConstantDouble("lm1", 2.5);
    public final Supplier<Double> l2 = calibrations.addConstantDouble("l2", 0.3);
    public final Supplier<Double> m2 = calibrations.addConstantDouble("m2", 4);
    public final Supplier<Double> lA = calibrations.addConstantDouble("la", 0.9);

    private final Namespace keepStablePIDNamespace = namespace.addChild("keep stable pid");
    private final Supplier<Double> keepStableKp = keepStablePIDNamespace.addConstantDouble("kP", 0.07);
    private final Supplier<Double> keepStableKi = keepStablePIDNamespace.addConstantDouble("kI", 0);
    private final Supplier<Double> keepStableKd = keepStablePIDNamespace.addConstantDouble("kD", 0);
    private final Supplier<Double> keepStableTolerance = keepStablePIDNamespace.addConstantDouble("tolerance", 0);
    private final Supplier<Double> keepStableWaitTime = keepStablePIDNamespace.addConstantDouble("wait time", 99999);
    public final PIDSettings keepStablePIDSettings = new PIDSettings(keepStableKp, keepStableKi, keepStableKd,
            keepStableTolerance, keepStableWaitTime);

    private double arbitraryFeedForward;

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

    private ArmFirstJoint(String namespaceName, CANSparkMax master, CANSparkMax slave) {
        super(namespaceName, master, slave);
        setIdleMode(CANSparkMax.IdleMode.kCoast);
        sparkMaxEncoder = master.getEncoder();
        absoluteEncoder = new DutyCycleEncoder(RobotMap.DIO.ARM_FIRST_JOINT_ABSOLUTE_ENCODER);
        configureEncoders();
        slave.follow(master, true);
        pidSettings = new PIDSettings(kP, kI, kD, tolerance, waitTime);
        feedForwardSettings = new FeedForwardSettings(kS, kV, kA, kG);
        trapezoidProfileSettings = new TrapezoidProfileSettings(maxVelocity, trapezoidAcceleration);
        configureDashboard();
    }

    @Override
    public void configureLoop(PIDSettings pidSettings, FeedForwardSettings feedForwardSettings,
                              TrapezoidProfileSettings trapezoidProfileSettings) {
        super.configureLoop(pidSettings, feedForwardSettings, trapezoidProfileSettings);
        configureEncoders();
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double setpoint, PIDSettings pidSettings,
                       FeedForwardSettings feedForwardSettings, TrapezoidProfileSettings trapezoidProfileSettings) {
        configPIDF(pidSettings, feedForwardSettings);
        configureTrapezoid(trapezoidProfileSettings);
        master.getPIDController().setReference(setpoint, controlMode.getSparkMaxControlType(), 0,
                arbitraryFeedForward, SparkMaxPIDController.ArbFFUnits.kVoltage);
    }

    public void setIdleMode(CANSparkMax.IdleMode idleMode) {
        master.setIdleMode(idleMode);
        slaves.get(0).setIdleMode(idleMode);
    }

    public void setVoltage(double voltage) {
        master.setVoltage(voltage);
    }

    public double getRelativePosition() {
        return sparkMaxEncoder.getPosition();
    }

    public double getAbsolutePosition() {
        if (absoluteEncoder.isConnected()) {
            double pos = absoluteEncoder.getAbsolutePosition()  * DEGREES_PER_ROTATION;
            if (pos > 90) pos -= 180;
            else pos += 180;
            return pos;
        }
        return getRelativePosition();
    }

    public boolean encoderConnected() {
        return absoluteEncoder.isConnected();
    }

    public CANSparkMax.IdleMode getIdleMode() {
        return master.getIdleMode();
    }

    public double getVelocity() {
        return sparkMaxEncoder.getVelocity();
    }

    public PIDSettings getPIDSettings() {
        return pidSettings;
    }

    public FeedForwardSettings getFeedForwardSettings() {
        return feedForwardSettings;
    }

    public TrapezoidProfileSettings getTrapezoidProfileSettings() {
        return trapezoidProfileSettings;
    }

    public void configureEncoders() {
        sparkMaxEncoder.setPositionConversionFactor(DEGREES_PER_ROTATION * GEAR_RATIO_MOTOR_TO_ABSOLUTE_ENCODER);
        sparkMaxEncoder.setVelocityConversionFactor(DEGREES_PER_ROTATION
                * GEAR_RATIO_MOTOR_TO_ABSOLUTE_ENCODER / SECONDS_IN_MINUTE);
        sparkMaxEncoder.setPosition(getAbsolutePosition());
//        absoluteEncoder.setPositionOffset(ABSOLUTE_ENCODER_OFFSET);
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("absolute encoder position", this::getAbsolutePosition);
        namespace.putNumber("spark max encoder position", this::getRelativePosition);
        namespace.putNumber("velocity", this::getVelocity);
        namespace.putData("test", new MoveSmartMotorControllerGenericSubsystem(this,
                pidSettings, feedForwardSettings, UnifiedControlMode.VELOCITY, () -> 0.0));
        namespace.putNumber("voltage", () -> master.getBusVoltage() * master.getAppliedOutput());
        namespace.putNumber("current", master::getOutputCurrent);
        namespace.putRunnable("set position", () -> sparkMaxEncoder.setPosition(getAbsolutePosition()));
        namespace.putNumber("aribtrary ff", () -> arbitraryFeedForward);
        namespace.putBoolean("absolute encoder connected", absoluteEncoder::isConnected);
    }

    public void setArbitraryFeedForward(double arbitraryFeedForward) {
        this.arbitraryFeedForward = arbitraryFeedForward;
//        this.arbitraryFeedForward = 0;
    }
}
