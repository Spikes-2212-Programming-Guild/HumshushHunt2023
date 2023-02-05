package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.spikes2212.command.drivetrains.smartmotorcontrollerdrivetrain.SparkMaxTankDrivetrain;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.Namespace;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.Supplier;

public class Drivetrain extends SparkMaxTankDrivetrain {

    public static Drivetrain instance;

    JoystickSim joystickSim;
    DifferentialDriveOdometry odometry;
    private final Namespace leftPIDNamespace = namespace.addChild("left pid");
    private Encoder m_leftEncoder = new Encoder(0, 1);
    private Encoder m_rightEncoder = new Encoder(2, 3);
    private EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    private EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);
    private Field2d m_field = new Field2d();
    private CANSparkMax m_leftMotor = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax m_rightMotor = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax m_leftSlave = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
    private CANSparkMax m_rightSlave = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
    private AnalogGyro m_gyro = new AnalogGyro(1);
    private AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);
    private final Supplier<Double> kPLeft = leftPIDNamespace.addConstantDouble("kP", 0.15);
    private final Supplier<Double> kILeft = leftPIDNamespace.addConstantDouble("kI", 0.1);
    private final Supplier<Double> kDLeft = leftPIDNamespace.addConstantDouble("kD", 0.25);
    private final Supplier<Double> waitTimeLeft = leftPIDNamespace.addConstantDouble("wait time", 1);
    private final Supplier<Double> toleranceLeft = leftPIDNamespace.addConstantDouble("tolerance", 0);
    private final PIDSettings leftPIDSettings;
    private final Namespace rightPIDNamespace = namespace.addChild("right pid");
    private final Supplier<Double> kPRight = rightPIDNamespace.addConstantDouble("kP", 0.15);
    private final Supplier<Double> kIRight = rightPIDNamespace.addConstantDouble("kI", 0.1);
    private final Supplier<Double> kDRight = rightPIDNamespace.addConstantDouble("kD", 0.25);
    private final Supplier<Double> waitTimeRight = rightPIDNamespace.addConstantDouble("wait time", 1);
    private final Supplier<Double> toleranceRight = rightPIDNamespace.addConstantDouble("tolerance", 0);
    private final PIDSettings rightPIDSettings;
    private final Namespace cameraPIDNamespace = namespace.addChild("camera pid");
    private final Supplier<Double> kPCamera = cameraPIDNamespace.addConstantDouble("kP", 0);
    private final Supplier<Double> kICamera = cameraPIDNamespace.addConstantDouble("kI", 0);
    private final Supplier<Double> kDCamera = cameraPIDNamespace.addConstantDouble("kD", 0);
    private final Supplier<Double> waitTimeCamera = cameraPIDNamespace.addConstantDouble("wait time", 0);
    private final Supplier<Double> toleranceCamera = cameraPIDNamespace.addConstantDouble("tolerance", 0);
    private final PIDSettings cameraPIDSettings;
    private final Namespace feedForwardNamespace = namespace.addChild("feed forward");
    private final Supplier<Double> kS = feedForwardNamespace.addConstantDouble("kS", 0);
    private final Supplier<Double> kV = feedForwardNamespace.addConstantDouble("kV", 0);
    private final Supplier<Double> kA = feedForwardNamespace.addConstantDouble("kA", 0);
    private final FeedForwardSettings feedForwardSettings;
    DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(
            DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
//            1/12.755,                    // 7.29:1 gearing reduction.
            12.755,
            7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
            45,                    // The mass of the robot is 60 kg.
            Units.inchesToMeters(3), // The robot uses 3" radius wheels.
            0.57,                  // The track width is 0.7112 meters.

            VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

    public Drivetrain(String namespaceName, CANSparkMax m_leftMotor, CANSparkMax m_rightMotor,
                      CANSparkMax m_leftSlave, CANSparkMax m_rightSlave) {
        super(namespaceName, m_leftMotor, m_rightMotor, m_leftSlave, m_rightSlave);
        leftPIDSettings = new PIDSettings(kPLeft, kILeft, kDLeft, waitTimeLeft, toleranceLeft);
        rightPIDSettings = new PIDSettings(kPRight, kIRight, kDRight, waitTimeRight, toleranceRight);
        cameraPIDSettings = new PIDSettings(kPCamera, kICamera, kDCamera, waitTimeCamera, toleranceCamera);
        feedForwardSettings = new FeedForwardSettings(kS, kV, kA);
        joystickSim = new JoystickSim(0);
        m_leftEncoder.setDistancePerPulse(2 * Math.PI * 3 * 0.0254 / 4096);
        m_rightEncoder.setDistancePerPulse(2 * Math.PI * 3 * 0.0254 / 4096);
        SmartDashboard.putData("Field", m_field);
        SmartDashboard.putNumber("left encoder", m_leftEncoder.get());
        SmartDashboard.putNumber("right encoder", m_rightEncoder.get());
        SmartDashboard.putNumber("left sim encoder", m_leftEncoderSim.getDistance());
        SmartDashboard.putNumber("right sim encoder", m_rightEncoderSim.getDistance());
        odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0);
    }

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain("drivetrain",
                    new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless),
                    new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless),
                    new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless),
                    new CANSparkMax(8, CANSparkMaxLowLevel.MotorType.kBrushless));
        }
        return instance;
    }

    public void periodic() {
        odometry.update(m_gyro.getRotation2d(),
                m_leftEncoder.getDistance(),
                m_rightEncoder.getDistance());
        m_field.setRobotPose(odometry.getPoseMeters());
    }

    public void setSpeeds(double leftSpeed, double rightSpeed) {
        setLeft(leftSpeed);
        setRight(rightSpeed);
    }

    @Override
    public void setLeft(double speedLeft) {
        m_leftMotor.set(speedLeft);
    }

    @Override
    public void setRight(double speedRight) {
        m_rightMotor.set(speedRight);
    }

    public PIDSettings getLeftPIDSettings() {
        return leftPIDSettings;
    }

    public PIDSettings getRightPIDSettings() {
        return rightPIDSettings;
    }

    public PIDSettings getCameraPIDSettings() {
        return cameraPIDSettings;
    }

    public FeedForwardSettings getFeedForwardSettings() {
        return feedForwardSettings;
    }

    public double getLeftEncoderPosition() {
        return m_leftEncoder.getDistance();
    }

    public double getRightEncoderPosition() {
        return m_rightEncoder.getDistance();
    }

    public void simulationPeriodic() {
        driveSim.setInputs(m_leftMotor.get() * RobotController.getInputVoltage(),
                m_rightMotor.get() * RobotController.getInputVoltage());

        driveSim.update(0.02);

        m_leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
        m_leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());
        m_rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
        m_rightEncoderSim.setRate(driveSim.getRightVelocityMetersPerSecond());
        m_gyroSim.setAngle(-driveSim.getHeading().getDegrees());
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("left encoder", m_leftEncoder::get);
        namespace.putNumber("right encoder", m_rightEncoder::get);
        namespace.putNumber("left sim encoder", m_leftEncoderSim::getDistance);
        namespace.putNumber("right sim encoder", m_rightEncoderSim::getDistance);

    }
}
