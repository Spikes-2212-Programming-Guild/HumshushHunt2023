package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.spikes2212.command.drivetrains.smartmotorcontrollerdrivetrain.SparkMaxTankDrivetrain;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.dashboard.Namespace;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotMap;

import java.util.function.Supplier;

public class Drivetrain extends SparkMaxTankDrivetrain {

    public static final double WHEEL_DIAMETER_IN_INCHES = -1;
    public static final double INCHES_TO_CM = -1;
    public static final double GEAR_RATIO = -1;
    public static final double DISTANCE_PER_PULSE = WHEEL_DIAMETER_IN_INCHES * INCHES_TO_CM * GEAR_RATIO * Math.PI;

    public static final double TRACK_WIDTH = -1;

    private static Drivetrain instance;

    private final AHRS gyro;

    private final DifferentialDriveOdometry odometry;
    private final DifferentialDriveKinematics kinematics;
    private final RamseteController ramseteController;
    private final Field2d field2d;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final Namespace trapezoidSettingsNamespace = namespace.addChild("trapezoid profile settings");
    private final Supplier<Double> trapezoidVelocity = trapezoidSettingsNamespace.addConstantDouble("velocity", 0);
    private final Supplier<Double> trapezoidAcceleration = trapezoidSettingsNamespace.addConstantDouble
            ("acceleration", 0);
    private final TrapezoidProfileSettings trapezoidProfileSettings;

    private final Namespace leftPIDNamespace = namespace.addChild("drive pid");
    private final Supplier<Double> kPLeft = leftPIDNamespace.addConstantDouble("kP", 0);
    private final Supplier<Double> kILeft = leftPIDNamespace.addConstantDouble("kI", 0);
    private final Supplier<Double> kDLeft = leftPIDNamespace.addConstantDouble("kD", 0);
    private final Supplier<Double> waitTimeLeft = leftPIDNamespace.addConstantDouble("wait time", 0);
    private final Supplier<Double> toleranceLeft = leftPIDNamespace.addConstantDouble("tolerance", 0);
    private final PIDSettings leftPIDSettings;

    private final Namespace rightPIDNamespace = namespace.addChild("drive pid");
    private final Supplier<Double> kPRight = rightPIDNamespace.addConstantDouble("kP", 0);
    private final Supplier<Double> kIRight = rightPIDNamespace.addConstantDouble("kI", 0);
    private final Supplier<Double> kDRight = rightPIDNamespace.addConstantDouble("kD", 0);
    private final Supplier<Double> waitTimeRight = rightPIDNamespace.addConstantDouble("wait time", 0);
    private final Supplier<Double> toleranceRight = rightPIDNamespace.addConstantDouble("tolerance", 0);
    private final PIDSettings rightPIDSettings;

    private final Namespace feedForwardNamespace = namespace.addChild("feed forward settings");
    private final Supplier<Double> kSFeedForward = feedForwardNamespace.addConstantDouble("kS", 0);
    private final Supplier<Double> kVFeedForward = feedForwardNamespace.addConstantDouble("kV", 0);
    private final Supplier<Double> kAFeedForward = feedForwardNamespace.addConstantDouble("kA", 0);
    private final Supplier<Double> kGFeedForward = feedForwardNamespace.addConstantDouble("kG", 0);
    private final FeedForwardSettings feedForwardSettings;

    private final Namespace anglePIDNamespace = namespace.addChild("angle pid");
    private final Supplier<Double> kPAngle = anglePIDNamespace.addConstantDouble("kP", 0);
    private final Supplier<Double> kIAngle = anglePIDNamespace.addConstantDouble("kI", 0);
    private final Supplier<Double> kDAngle = anglePIDNamespace.addConstantDouble("kD", 0);
    private final Supplier<Double> waitTimeAngle = anglePIDNamespace.addConstantDouble("wait time", 0);
    private final Supplier<Double> toleranceAngle = anglePIDNamespace.addConstantDouble("tolerance", 0);
    private final PIDSettings anglePIDSettings;

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain(
                    "drivetrain",
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

    private Drivetrain(String namespaceName, CANSparkMax leftMaster, CANSparkMax leftSlave,
                       CANSparkMax rightMaster, CANSparkMax rightSlave) {
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
        this.gyro = new AHRS();
        this.leftEncoder = leftMaster.getEncoder();
        this.rightEncoder = rightMaster.getEncoder();
        this.trapezoidProfileSettings = new TrapezoidProfileSettings(trapezoidVelocity, trapezoidAcceleration);
        this.leftPIDSettings = new PIDSettings(kPLeft, kILeft, kDLeft, waitTimeLeft, toleranceLeft);
        this.rightPIDSettings = new PIDSettings(kPRight, kIRight, kDRight, waitTimeRight, toleranceRight);
        this.feedForwardSettings = new FeedForwardSettings(kSFeedForward, kVFeedForward, kAFeedForward, kGFeedForward);
        this.anglePIDSettings = new PIDSettings(kPAngle, kIAngle, kDAngle, waitTimeAngle, toleranceAngle);
        configureDashboard();
        this.odometry = new DifferentialDriveOdometry(new Rotation2d(gyro.getYaw()), // @todo make sure this is correct
                getLeftEncoderPosition(), getRightEncoderPosition());
        this.kinematics = new DifferentialDriveKinematics(TRACK_WIDTH);
        this.ramseteController = new RamseteController();
        this.field2d = new Field2d();
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public void resetGyro() {
        gyro.reset();
    }

    public PIDSettings getLeftPIDSettings() {
        return leftPIDSettings;
    }

    public PIDSettings getRightPIDSettings() {
        return rightPIDSettings;
    }

    public PIDSettings getAnglePIDSettings() {
        return anglePIDSettings;
    }

    public double getLeftEncoderPosition() {
        return leftEncoder.getPosition();
    }

    public double getRightEncoderPosition() {
        return rightEncoder.getPosition();
    }

    public double getLeftSpeed() {
        return leftEncoder.getVelocity();
    }

    public double getRightSpeed() {
        return rightEncoder.getVelocity();
    }

    public Pose2d getPose2d() {
        return this.odometry.getPoseMeters();
    }

    public DifferentialDriveOdometry getOdometry() {
        return this.odometry;
    }

    public DifferentialDriveKinematics getKinematics() {
        return this.kinematics;
    }

    public RamseteController getRamseteController() {
        return this.ramseteController;
    }

    @Override
    public void configureLoop(PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                              FeedForwardSettings feedForwardSettings,
                              TrapezoidProfileSettings trapezoidProfileSettings) {
        super.configureLoop(leftPIDSettings, rightPIDSettings, feedForwardSettings, trapezoidProfileSettings);
        leftEncoder.setPositionConversionFactor(DISTANCE_PER_PULSE);
        leftEncoder.setVelocityConversionFactor(DISTANCE_PER_PULSE / 60);
        rightEncoder.setPositionConversionFactor(DISTANCE_PER_PULSE);
        rightEncoder.setVelocityConversionFactor(DISTANCE_PER_PULSE / 60);
    }

    @Override
    public void configureDashboard() {
        namespace.putData("reset encoders", new InstantCommand(this::resetEncoders).ignoringDisable(true));
        namespace.putNumber("left neo 1 encoder value", this::getLeftEncoderPosition);
        namespace.putNumber("right neo 1 encoder value", this::getRightEncoderPosition);
    }
}
