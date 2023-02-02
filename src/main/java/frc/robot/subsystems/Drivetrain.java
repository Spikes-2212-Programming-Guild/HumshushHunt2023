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
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotMap;

import java.util.function.Supplier;

public class Drivetrain extends SparkMaxTankDrivetrain {

    public static final double WHEEL_DIAMETER_IN_INCHES = 6;
    public static final double GEAR_RATIO = 1 / 12.755;
    public static final double INCHES_TO_METERS = 0.0254;
    public static final double DISTANCE_PER_PULSE = WHEEL_DIAMETER_IN_INCHES * GEAR_RATIO * Math.PI * INCHES_TO_METERS;

    public static final double TRACK_WIDTH = 0.57;

    public static final int SECONDS_IN_MINUTE = 60;

    private static Drivetrain instance;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final AHRS gyro;

    private final DifferentialDriveOdometry odometry;
    private final DifferentialDriveKinematics kinematics;
    private final RamseteController ramseteController;
    private final Field2d field2d;

    private final Namespace leftPIDNamespace = namespace.addChild("left pid");
    private final Supplier<Double> kPLeft = leftPIDNamespace.addConstantDouble("kP", 0);
    private final Supplier<Double> kILeft = leftPIDNamespace.addConstantDouble("kI", 0);
    private final Supplier<Double> kDLeft = leftPIDNamespace.addConstantDouble("kD", 0);
    private final Supplier<Double> waitTimeLeft = leftPIDNamespace.addConstantDouble("wait time", 0);
    private final Supplier<Double> toleranceLeft = leftPIDNamespace.addConstantDouble("tolerance", 0);
    private final PIDSettings leftPIDSettings;

    private final Namespace rightPIDNamespace = namespace.addChild("right pid");
    private final Supplier<Double> kPRight = rightPIDNamespace.addConstantDouble("kP", 0);
    private final Supplier<Double> kIRight = rightPIDNamespace.addConstantDouble("kI", 0);
    private final Supplier<Double> kDRight = rightPIDNamespace.addConstantDouble("kD", 0);
    private final Supplier<Double> waitTimeRight = rightPIDNamespace.addConstantDouble("wait time", 0);
    private final Supplier<Double> toleranceRight = rightPIDNamespace.addConstantDouble("tolerance", 0);
    private final PIDSettings rightPIDSettings;

    private final Namespace anglePIDNamespace = namespace.addChild("angle pid");
    private final Supplier<Double> kPAngle = anglePIDNamespace.addConstantDouble("kP", 0);
    private final Supplier<Double> kIAngle = anglePIDNamespace.addConstantDouble("kI", 0);
    private final Supplier<Double> kDAngle = anglePIDNamespace.addConstantDouble("kD", 0);
    private final Supplier<Double> waitTimeAngle = anglePIDNamespace.addConstantDouble("wait time", 0);
    private final Supplier<Double> toleranceAngle = anglePIDNamespace.addConstantDouble("tolerance", 0);
    private final PIDSettings anglePIDSettings;

    private final Namespace feedForwardNamespace = namespace.addChild("feed forward");
    private final Supplier<Double> kS = feedForwardNamespace.addConstantDouble("kS", 0);
    private final Supplier<Double> kV = feedForwardNamespace.addConstantDouble("kV", 0);
    private final Supplier<Double> kA = feedForwardNamespace.addConstantDouble("kA", 0);
    private final FeedForwardSettings feedForwardSettings;

    private final Namespace trapezoidProfile = namespace.addChild("trapezoid profile settings");
    private final Supplier<Double> maxVelocity = trapezoidProfile.addConstantDouble("max velocity", 0);
    private final Supplier<Double> trapezoidAcceleration = trapezoidProfile.addConstantDouble
            ("acceleration", 0);
    private final TrapezoidProfileSettings trapezoidProfileSettings;

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain(
                    "drivetrain",
                    new CANSparkMax(RobotMap.CAN.DRIVETRAIN_LEFT_SPARKMAX_MASTER,
                            CANSparkMaxLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.DRIVETRAIN_LEFT_SPARKMAX_SLAVE,
                            CANSparkMaxLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.DRIVETRAIN_RIGHT_SPARKMAX_MASTER,
                            CANSparkMaxLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.DRIVETRAIN_RIGHT_SPARKMAX_SLAVE,
                            CANSparkMaxLowLevel.MotorType.kBrushless));
        }
        return instance;
    }

    private Drivetrain(String namespaceName, CANSparkMax leftMaster, CANSparkMax leftSlave,
                       CANSparkMax rightMaster, CANSparkMax rightSlave) {
        super(
                namespaceName, leftMaster, leftSlave, rightMaster, rightSlave);
        gyro = new AHRS();
        leftEncoder = leftMaster.getEncoder();
        rightEncoder = rightMaster.getEncoder();
        setConversionFactors();
        leftPIDSettings = new PIDSettings(kPLeft, kILeft, kDLeft, waitTimeLeft, toleranceLeft);
        rightPIDSettings = new PIDSettings(kPRight, kIRight, kDRight, waitTimeRight, toleranceRight);
        anglePIDSettings = new PIDSettings(kPAngle, kIAngle, kDAngle, waitTimeAngle, toleranceAngle);
        trapezoidProfileSettings = new TrapezoidProfileSettings(maxVelocity, trapezoidAcceleration);
        feedForwardSettings = new FeedForwardSettings(kS, kV, kA);
        odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), getLeftPosition(), getRightPosition());
        kinematics = new DifferentialDriveKinematics(TRACK_WIDTH);
        ramseteController = new RamseteController();
        field2d = new Field2d();
        configureDashboard();
    }

    @Override
    public void configureLoop(PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                              FeedForwardSettings feedForwardSettings,
                              TrapezoidProfileSettings trapezoidProfileSettings) {
        super.configureLoop(leftPIDSettings, rightPIDSettings, feedForwardSettings, trapezoidProfileSettings);
        setConversionFactors();
    }

    @Override
    public void periodic() {
        super.periodic();
        odometry.update(gyro.getRotation2d(), getLeftPosition(), getRightPosition());
        field2d.setRobotPose(getPose2d());
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public void resetOdometry(Pose2d pose2d) {
        odometry.resetPosition(gyro.getRotation2d(), getLeftPosition(), getRightPosition(), pose2d);
    }

    public void resetGyro() {
        gyro.reset();
    }

    public double getLeftPosition() {
        return leftEncoder.getPosition();
    }

    public double getRightPosition() {
        return rightEncoder.getPosition();
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public double getYaw() {
        return gyro.getYaw();
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

    public double getLeftSpeed() {
        return leftEncoder.getVelocity();
    }

    public double getRightSpeed() {
        return rightEncoder.getVelocity();
    }

    public Pose2d getPose2d() {
        return odometry.getPoseMeters();
    }

    public FeedForwardSettings getFeedForwardSettings() {
        return feedForwardSettings;
    }

    public TrapezoidProfileSettings getTrapezoidProfileSettings() {
        return trapezoidProfileSettings;
    }

    public DifferentialDriveOdometry getOdometry() {
        return odometry;
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    public RamseteController getRamseteController() {
        return ramseteController;
    }

    private void setConversionFactors() {
        leftEncoder.setPositionConversionFactor(DISTANCE_PER_PULSE);
        leftEncoder.setVelocityConversionFactor(DISTANCE_PER_PULSE / SECONDS_IN_MINUTE);
        rightEncoder.setPositionConversionFactor(DISTANCE_PER_PULSE);
        rightEncoder.setVelocityConversionFactor(DISTANCE_PER_PULSE / SECONDS_IN_MINUTE);
    }

    @Override
    public void configureDashboard() {
        namespace.putData("reset encoders", new InstantCommand(this::resetEncoders).ignoringDisable(true));
        namespace.putData("reset gyro", new InstantCommand(this::resetGyro).ignoringDisable(true));
        namespace.putData("field2d", field2d);
        namespace.putNumber("left position", this::getLeftPosition);
        namespace.putNumber("right position", this::getRightPosition);
        namespace.putNumber("gyro yaw", gyro::getYaw);
    }
}
