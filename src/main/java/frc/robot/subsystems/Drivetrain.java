package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.spikes2212.command.drivetrains.smartmotorcontrollerdrivetrain.SparkMaxTankDrivetrain;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.Namespace;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotMap;

import java.util.function.Supplier;

public class Drivetrain extends SparkMaxTankDrivetrain {

    public static final double WHEEL_DIAMETER_IN_INCHES = -1;
    public static final double INCHES_TO_CM = -1;
    public static final double GEAR_RATIO = -1;
    public static final double DISTANCE_PER_PULSE = WHEEL_DIAMETER_IN_INCHES * INCHES_TO_CM * GEAR_RATIO * Math.PI;

    private static Drivetrain instance;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final Namespace drivePIDNamespace = namespace.addChild("drive pid");
    private final Supplier<Double> kPDrive = drivePIDNamespace.addConstantDouble("kP", 0);
    private final Supplier<Double> kIDrive = drivePIDNamespace.addConstantDouble("kI", 0);
    private final Supplier<Double> kDDrive = drivePIDNamespace.addConstantDouble("kD", 0);
    private final Supplier<Double> waitTimeDrive = drivePIDNamespace.addConstantDouble("wait time", 0);
    private final Supplier<Double> toleranceDrive = drivePIDNamespace.addConstantDouble("tolerance", 0);
    private final PIDSettings drivePIDSettings;

    private final Namespace anglePIDNamespace = namespace.addChild("angle pid");
    private final Supplier<Double> kPAngle = drivePIDNamespace.addConstantDouble("kP", 0);
    private final Supplier<Double> kIAngle = drivePIDNamespace.addConstantDouble("kI", 0);
    private final Supplier<Double> kDAngle = drivePIDNamespace.addConstantDouble("kD", 0);
    private final Supplier<Double> waitTimeAngle = drivePIDNamespace.addConstantDouble("wait time", 0);
    private final Supplier<Double> toleranceAngle = drivePIDNamespace.addConstantDouble("tolerance", 0);
    private final PIDSettings anglePIDSettings;

    //@todo add navx

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain(
                    new CANSparkMax(RobotMap.CAN.DRIVETRAIN_LEFT_SPARKMAX_1,
                            CANSparkMaxLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.DRIVETRAIN_LEFT_SPARKMAX_2,
                            CANSparkMaxLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.DRIVETRAIN_RIGHT_SPARKMAX_1,
                            CANSparkMaxLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.DRIVETRAIN_RIGHT_SPARKMAX_2,
                            CANSparkMaxLowLevel.MotorType.kBrushless));
        }
        return instance;
    }

    private Drivetrain(CANSparkMax left1, CANSparkMax left2, CANSparkMax right1, CANSparkMax right2) {
        super(
                "drivetrain",
                new CANSparkMax(RobotMap.CAN.DRIVETRAIN_LEFT_SPARKMAX_1,
                        CANSparkMaxLowLevel.MotorType.kBrushless),
                new CANSparkMax(RobotMap.CAN.DRIVETRAIN_LEFT_SPARKMAX_2,
                        CANSparkMaxLowLevel.MotorType.kBrushless),
                new CANSparkMax(RobotMap.CAN.DRIVETRAIN_RIGHT_SPARKMAX_1,
                        CANSparkMaxLowLevel.MotorType.kBrushless),
                new CANSparkMax(RobotMap.CAN.DRIVETRAIN_RIGHT_SPARKMAX_2,
                        CANSparkMaxLowLevel.MotorType.kBrushless));
        this.leftEncoder = left1.getEncoder();
        this.rightEncoder = right1.getEncoder();
        leftEncoder.setPositionConversionFactor(DISTANCE_PER_PULSE);
        rightEncoder.setPositionConversionFactor(DISTANCE_PER_PULSE);
        this.drivePIDSettings = new PIDSettings(kPDrive, kIDrive, kDDrive, waitTimeDrive, toleranceDrive);
        this.anglePIDSettings = new PIDSettings(kPAngle, kIAngle, kDAngle, waitTimeAngle, toleranceAngle);
        configureDashboard();
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public PIDSettings getDrivePIDSettings() {
        return drivePIDSettings;
    }

    public double getLeftEncoderPosition() {
        return leftEncoder.getPosition();
    }

    public double getRightEncoderPosition() {
        return -rightEncoder.getPosition();
    }

    @Override
    public void configureDashboard() {
        namespace.putData("reset encoders", new InstantCommand(this::resetEncoders) {
            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        });
        namespace.putNumber("left neo 1 encoder value", this::getLeftEncoderPosition);
        namespace.putNumber("right neo 1 encoder value", this::getRightEncoderPosition);
    }
}
