package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.spikes2212.command.drivetrains.smartmotorcontrollerdrivetrain.SparkMaxTankDrivetrain;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.dashboard.Namespace;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private final DifferentialDrivetrainSim drivetrainSim;

    private final Encoder leftEncoder;
    private final Encoder rightEncoder;

    private final ADXRS450_Gyro gyro;

    private final EncoderSim leftEncoderSim;
    private final EncoderSim rightEncoderSim;

    private final ADXRS450_GyroSim gyroSim;

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
        this.gyro = new ADXRS450_Gyro();
        this.leftEncoder = new Encoder(0, 1);
        this.rightEncoder = new Encoder(2, 3);
        this.gyroSim = new ADXRS450_GyroSim(gyro);
        this.leftEncoderSim = new EncoderSim(leftEncoder);
        this.rightEncoderSim = new EncoderSim(rightEncoder);
        this.drivetrainSim = new DifferentialDrivetrainSim(
                DCMotor.getNEO(2),
                1 / 12.755,
                1,
                10,
                INCHES_TO_METERS * 3,
                0.57,
                VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005)
        );
        this.setConversionFactors();
        this.leftPIDSettings = new PIDSettings(kPLeft, kILeft, kDLeft, waitTimeLeft, toleranceLeft);
        this.rightPIDSettings = new PIDSettings(kPRight, kIRight, kDRight, waitTimeRight, toleranceRight);
        this.anglePIDSettings = new PIDSettings(kPAngle, kIAngle, kDAngle, waitTimeAngle, toleranceAngle);
        this.trapezoidProfileSettings = new TrapezoidProfileSettings(maxVelocity, trapezoidAcceleration);
        this.feedForwardSettings = new FeedForwardSettings(kS, kV, kA);
        this.odometry = new DifferentialDriveOdometry(gyro.getRotation2d(),
                getLeftPosition(), getRightPosition());
        this.kinematics = new DifferentialDriveKinematics(TRACK_WIDTH);
        this.ramseteController = new RamseteController();
        this.field2d = new Field2d();
        SmartDashboard.putData("field 2d", field2d);
        configureDashboard();
    }

    @Override
    public void configureLoop(PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                              FeedForwardSettings feedForwardSettings,
                              TrapezoidProfileSettings trapezoidProfileSettings) {
        super.configureLoop(leftPIDSettings, rightPIDSettings, feedForwardSettings, trapezoidProfileSettings);
        this.setConversionFactors();
    }

    @Override
    public void periodic() {
        super.periodic();
        odometry.update(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
        field2d.setRobotPose(odometry.getPoseMeters());
    }

    @Override
    public void simulationPeriodic() {
        drivetrainSim.setInputs(leftMaster.get() * RobotController.getInputVoltage(),
                rightMaster.get() * RobotController.getInputVoltage());

        drivetrainSim.update(0.02);

        leftEncoderSim.setDistance(drivetrainSim.getLeftPositionMeters());
        leftEncoderSim.setRate(drivetrainSim.getLeftVelocityMetersPerSecond());
        rightEncoderSim.setDistance(drivetrainSim.getRightPositionMeters());
        rightEncoderSim.setRate(drivetrainSim.getRightVelocityMetersPerSecond());
        gyroSim.setAngle(-drivetrainSim.getHeading().getDegrees());
    }

    private void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    private void resetOdometry(Pose2d pose2d) {
        odometry.resetPosition(gyro.getRotation2d(), 0, 0, pose2d);
    }

    private void resetGyro() {
        gyro.reset();
    }

    private void setConversionFactors() {
        leftEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
    }

    public double getLeftPosition() {
        return leftEncoder.getDistance();
    }

    public double getRightPosition() {
        return rightEncoder.getDistance();
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
        return leftEncoder.getRate();
    }

    public double getRightSpeed() {
        return rightEncoder.getRate();
    }

    public Pose2d getPose2d() {
        return this.odometry.getPoseMeters();
    }

    public FeedForwardSettings getFeedForwardSettings() {
        return this.feedForwardSettings;
    }

    public TrapezoidProfileSettings getTrapezoidProfileSettings() {
        return this.trapezoidProfileSettings;
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
    public void configureDashboard() {
        namespace.putData("reset encoders", new InstantCommand(this::resetEncoders).ignoringDisable(true));
        namespace.putData("reset gyro", new InstantCommand(this::resetGyro).ignoringDisable(true));
        namespace.putData("field2d", field2d);
        namespace.putNumber("left neo 1 encoder value", this::getLeftPosition);
        namespace.putNumber("right neo 1 encoder value", this::getRightPosition);
        namespace.putNumber("gyro yaw", gyro::getAngle);
        namespace.putNumber("left master value", leftMaster::get);
        namespace.putNumber("x", () -> getPose2d().getX());
        namespace.putNumber("y", () -> getPose2d().getY());
    }
}
