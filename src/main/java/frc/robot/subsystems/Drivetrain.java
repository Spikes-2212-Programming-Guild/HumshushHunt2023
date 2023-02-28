package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.spikes2212.command.drivetrains.commands.DriveArcade;
import com.spikes2212.command.drivetrains.commands.DriveTank;
import com.spikes2212.command.drivetrains.commands.DriveTankWithPID;
import com.spikes2212.command.drivetrains.smartmotorcontrollerdrivetrain.SparkMaxTankDrivetrain;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.TrapezoidProfileSettings;
import com.spikes2212.dashboard.Namespace;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotMap;

import java.util.function.Supplier;

public class Drivetrain extends SparkMaxTankDrivetrain {

    private static final double WHEEL_DIAMETER_IN_INCHES = 6;
    private static final double GEAR_RATIO = 1 / 11.16;
    private static final double INCHES_TO_METERS = 0.0254;
    private static final double DISTANCE_PER_ROTATION = WHEEL_DIAMETER_IN_INCHES * GEAR_RATIO * Math.PI * INCHES_TO_METERS;

    private static final double TRACK_WIDTH = 0.57;

    private static final int SECONDS_IN_MINUTE = 60;

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

    private final Namespace cameraPIDNamespace = namespace.addChild("camera pid");
    private final Supplier<Double> kPCamera = cameraPIDNamespace.addConstantDouble("kP", 0.04);
    private final Supplier<Double> kICamera = cameraPIDNamespace.addConstantDouble("kI", 0.0001);
    private final Supplier<Double> kDCamera = cameraPIDNamespace.addConstantDouble("kD", 0.005);
    private final Supplier<Double> waitTimeCamera = cameraPIDNamespace.addConstantDouble("wait time", 0.5);
    private final Supplier<Double> toleranceCamera = cameraPIDNamespace.addConstantDouble("tolerance", 1);
    private final PIDSettings cameraPIDSettings;

    private final Namespace climbPIDNamespace = namespace.addChild("climb pid");
    private final Supplier<Double> kP = climbPIDNamespace.addConstantDouble("kP", 0.05);
    private final Supplier<Double> kI = climbPIDNamespace.addConstantDouble("kI", 0);
    private final Supplier<Double> kD = climbPIDNamespace.addConstantDouble("kD", 0.025);
    private final Supplier<Double> tolerance = climbPIDNamespace.addConstantDouble("tolerance", 0);
    private final Supplier<Double> waitTime = climbPIDNamespace.addConstantDouble("wait time", 1);
    public final PIDSettings climbPIDSettings = new PIDSettings(kP, kI, kD, tolerance, waitTime);

    private final Namespace feedForwardNamespace = namespace.addChild("feed forward");
    private final Supplier<Double> kS = feedForwardNamespace.addConstantDouble("kS", 0.25); //0.14
    private final Supplier<Double> kV = feedForwardNamespace.addConstantDouble("kV", 3); //0.28
    private final Supplier<Double> kA = feedForwardNamespace.addConstantDouble("kA", 4);
    private final Supplier<Double> limelightkS = feedForwardNamespace.addConstantDouble("limelight kS", 0.14);
    private final FeedForwardSettings feedForwardSettings;

    private final Namespace trapezoidProfileNamespace = namespace.addChild("trapezoid profile settings");
    private final Supplier<Double> maxVelocity = trapezoidProfileNamespace.addConstantDouble("max velocity", 0);
    private final Supplier<Double> trapezoidAcceleration = trapezoidProfileNamespace.addConstantDouble
            ("acceleration", 0);
    private final TrapezoidProfileSettings trapezoidProfileSettings;

    private final Namespace testing = namespace.addChild("testing");
    private final Supplier<Double> leftSetpoint = testing.addConstantDouble("left setpoint", 0);
    private final Supplier<Double> rightSetpoint = testing.addConstantDouble("right setpoint", 0);

    private double prevLeftSetpoint = 0;
    private double prevRightSetpoint = 0;
    private double maxAcceleration = 0;
    private double maxLeftSpeed = 0;
    private double maxRightSpeed = 0;

    private Drivetrain(String namespaceName, CANSparkMax leftMaster, CANSparkMax leftSlave,
                       CANSparkMax rightMaster, CANSparkMax rightSlave, AHRS gyro, double trackWidth) {
        super(
                namespaceName, leftMaster, leftSlave, rightMaster, rightSlave);
        this.gyro = gyro;
        leftEncoder = leftMaster.getEncoder();
        //yes i'm serious
        rightController.setInverted(false);
        rightMaster.setInverted(true);
        rightEncoder = rightMaster.getEncoder();
        configureEncoders();
        leftPIDSettings = new PIDSettings(kPLeft, kILeft, kDLeft, toleranceLeft, waitTimeLeft);
        rightPIDSettings = new PIDSettings(kPRight, kIRight, kDRight, toleranceRight, waitTimeRight);
        cameraPIDSettings = new PIDSettings(kPCamera, kICamera, kDCamera, toleranceCamera, waitTimeCamera);
        trapezoidProfileSettings = new TrapezoidProfileSettings(maxVelocity, trapezoidAcceleration);
        feedForwardSettings = new FeedForwardSettings(kS, kV, kA);
        odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), getLeftPosition(), getRightPosition());
        kinematics = new DifferentialDriveKinematics(trackWidth);
        ramseteController = new RamseteController();
        field2d = new Field2d();
        configureDashboard();
    }

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
                            CANSparkMaxLowLevel.MotorType.kBrushless),
                    new AHRS(SerialPort.Port.kUSB),
                    TRACK_WIDTH);
        }
        return instance;
    }

    @Override
    public void configureLoop(PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                              FeedForwardSettings feedForwardSettings,
                              TrapezoidProfileSettings trapezoidProfileSettings) {
        super.configureLoop(leftPIDSettings, rightPIDSettings, feedForwardSettings, trapezoidProfileSettings);
        configureEncoders();
        rightMaster.setInverted(true);
        prevLeftSetpoint = 0;
        prevRightSetpoint = 0;
    }

    @Override
    public void periodic() {
        super.periodic();
        odometry.update(gyro.getRotation2d(), getLeftPosition(), getRightPosition());
        field2d.setRobotPose(getPose2d()); //I think Field2d coordinate system has (0,0) at top/bottom left
        //so we might need to do a minus here @TODO check this
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public void resetOdometry(Pose2d pose2d) {
        odometry.resetPosition(gyro.getRotation2d(), getLeftPosition(), getRightPosition(), pose2d);
    }

    public void setMode(CANSparkMax.IdleMode mode) {
        leftMaster.setIdleMode(mode);
        leftSlaves.get(0).setIdleMode(mode);
        rightMaster.setIdleMode(mode);
        rightSlaves.get(0).setIdleMode(mode);
    }

    public void resetGyro() {
        gyro.reset();
    }


    public void tankDriveVoltages(double leftVoltage, double rightVoltage) {
        double leftInput = Math.signum(leftVoltage) * Math.sqrt(Math.abs(leftVoltage / RobotController.getBatteryVoltage()));
        double rightInput = Math.signum(rightVoltage) * Math.sqrt(Math.abs(rightVoltage / RobotController.getBatteryVoltage()));
        tankDrive(leftInput, rightInput);
    }

    @Override
    public void pidSet(UnifiedControlMode controlMode, double leftSetpoint, double rightSetpoint,
                       PIDSettings leftPIDSettings, PIDSettings rightPIDSettings,
                       FeedForwardSettings feedForwardSettings) {
        configPIDF(leftPIDSettings, rightPIDSettings, feedForwardSettings);
        configureTrapezoid(trapezoidProfileSettings);
        leftMaster.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 20);
        rightMaster.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 20);
        leftMaster.getPIDController().setReference(leftSetpoint, controlMode.getSparkMaxControlType(),
                0, feedForwardSettings.getkS() * Math.signum(leftSetpoint)
                        + kA.get() * (leftSetpoint - prevLeftSetpoint) / 0.02);
        rightMaster.getPIDController().setReference(rightSetpoint, controlMode.getSparkMaxControlType(),
                0, feedForwardSettings.getkS() * Math.signum(rightSetpoint)
                        + kA.get() * (rightSetpoint - prevRightSetpoint) / 0.02);
        prevLeftSetpoint = leftSetpoint;
        prevRightSetpoint = rightSetpoint;
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
        return gyro.getAngle();
    }

    public double getLeftSpeed() {
        double speed = leftEncoder.getVelocity();
        maxLeftSpeed = Math.max(speed, maxLeftSpeed);
        return speed;
    }

    public double getRightSpeed() {
        double speed = rightEncoder.getVelocity();
        maxRightSpeed = Math.max(maxRightSpeed, speed);
        return speed;
    }

    public Pose2d getPose2d() {
        return odometry.getPoseMeters();
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

    private void configureEncoders() {
        leftEncoder.setPositionConversionFactor(DISTANCE_PER_ROTATION);
        leftEncoder.setVelocityConversionFactor(DISTANCE_PER_ROTATION / SECONDS_IN_MINUTE);
        rightEncoder.setPositionConversionFactor(DISTANCE_PER_ROTATION);
        rightEncoder.setVelocityConversionFactor(DISTANCE_PER_ROTATION / SECONDS_IN_MINUTE);
    }

    private double getAcceleration() {
        double accel = gyro.getWorldLinearAccelZ();
        maxAcceleration = Math.max(accel, maxAcceleration);
        return accel;
    }

    private double getPoseX() {
        return getPose2d().getX();
    }

    private double getPoseY() {
        return getPose2d().getY();
    }

    @Override
    public void configureDashboard() {
        namespace.putData("reset encoders", new InstantCommand(this::resetEncoders).ignoringDisable(true));
        namespace.putData("reset gyro", new InstantCommand(this::resetGyro).ignoringDisable(true));
        namespace.putData("field2d", field2d);
        namespace.putNumber("left position", this::getLeftPosition);
        namespace.putNumber("right position", this::getRightPosition);
        namespace.putNumber("left velocity", this::getLeftSpeed);
        namespace.putNumber("right velocity", this::getRightSpeed);
        namespace.putNumber("yaw", this::getYaw);
        namespace.putNumber("pitch", this::getPitch);
        namespace.putNumber("pose x", this::getPoseX);
        namespace.putNumber("pose y", this::getPoseY);
        namespace.putNumber("accel", this::getAcceleration);
        namespace.putNumber("max acceleration", () -> maxAcceleration);
        namespace.putNumber("max left speed", () -> maxLeftSpeed);
        namespace.putNumber("max right speed", () -> maxRightSpeed);
        Supplier<Double> val = namespace.addConstantDouble("ks value", 0);
        namespace.putData("test ks", new DriveTank(this, val, val));
        namespace.putNumber("left output", leftMaster::getAppliedOutput);
        namespace.putNumber("right output", rightMaster::getAppliedOutput);
        namespace.putNumber("kG current value", feedForwardSettings::getkG);
        namespace.putData("turn", new DriveArcade(this, () -> 0.0, () -> (limelightkS.get() / RobotController.getBatteryVoltage())));
        namespace.putData("drive straight", new DriveTankWithPID(this, leftPIDSettings, rightPIDSettings, leftSetpoint,
                rightSetpoint, this::getLeftSpeed, this::getRightSpeed, feedForwardSettings, feedForwardSettings) {
            @Override
            public void execute() {
                leftPIDController.setSetpoint(leftSetpoint.get());
                rightPIDController.setSetpoint(rightSetpoint.get());
                leftPIDController.setTolerance(leftPIDSettings.getTolerance());
                rightPIDController.setTolerance(rightPIDSettings.getTolerance());
                leftPIDController.setPID(leftPIDSettings.getkP(), leftPIDSettings.getkI(), leftPIDSettings.getkD());
                rightPIDController.setPID(rightPIDSettings.getkP(), rightPIDSettings.getkI(), rightPIDSettings.getkD());
                leftFeedForwardController.setGains(leftFeedForwardSettings.getkS(), leftFeedForwardSettings.getkV(),
                        leftFeedForwardSettings.getkA(), leftFeedForwardSettings.getkG());
                rightFeedForwardController.setGains(rightFeedForwardSettings.getkS(), rightFeedForwardSettings.getkV(),
                        rightFeedForwardSettings.getkA(), rightFeedForwardSettings.getkG());
                ((Drivetrain) drivetrain).tankDriveVoltages((leftPIDController.calculate(leftSource.get()) +
                                leftFeedForwardController.calculate(leftSetpoint.get())),
                        rightPIDController.calculate(rightSource.get()) +
                                rightFeedForwardController.calculate(rightSetpoint.get()));
            }
        });
        double batteryVoltage = RobotController.getBatteryVoltage();
        namespace.putData("drive in voltages", run(() -> tankDriveVoltages(leftSetpoint.get(), rightSetpoint.get())));
        namespace.putNumber("battery voltage", batteryVoltage);
        namespace.putNumber("left voltage", () -> leftMaster.getAppliedOutput() * leftMaster.getBusVoltage());
        namespace.putNumber("right voltage", () -> rightMaster.getAppliedOutput() * rightMaster.getBusVoltage());
        namespace.putData("do weird thing", new RunCommand(() -> setLeft(3.428 / RobotController.getBatteryVoltage()), this));
        namespace.putData("accelerate", new FunctionalCommand(() -> {
        }, () -> {
        }, b -> {
        }, () -> false, this) {
            private DriveArcade driveArcade;

            @Override
            public void initialize() {
                double now = Timer.getFPGATimestamp();
                driveArcade = new DriveArcade(Drivetrain.getInstance(), () -> now + 0.5 * (Timer.getFPGATimestamp() - now),
                        () -> 0.0);
            }

            @Override
            public void execute() {
                driveArcade.execute();
            }

            @Override
            public void end(boolean interrupted) {
                driveArcade.end(interrupted);
            }
        });
    }
}
