package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.spikes2212.command.drivetrains.TankDrivetrain;
import com.spikes2212.command.drivetrains.smartmotorcontrollerdrivetrain.SparkMaxTankDrivetrain;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.dashboard.ChildNamespace;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.RobotMap;

import java.util.function.Supplier;

public class Drivetrain extends SparkMaxTankDrivetrain {

    private static Drivetrain instance;

    ChildNamespace drivePID = (ChildNamespace) namespace.addChild("drive pid");
    private final Supplier<Double> kPDrive = drivePID.addConstantDouble("kP", 0);
    private final Supplier<Double> kIDrive = drivePID.addConstantDouble("kI", 0);
    private final Supplier<Double> kDDrive = drivePID.addConstantDouble("kD", 0);
    private final Supplier<Double> toleranceDrive = drivePID.addConstantDouble("tolerance", 0);
    private final Supplier<Double> waitTimeDrive = drivePID.addConstantDouble("wait time", 0);
    private final PIDSettings drivePIDSettings = new PIDSettings(kPDrive, kIDrive, kDDrive, toleranceDrive, waitTimeDrive);

    private final ChildNamespace anglePID = (ChildNamespace) namespace.addChild("angle pid");
    private final Supplier<Double> kPAngle = anglePID.addConstantDouble("kP", 0);
    private final Supplier<Double> kIAngle = anglePID.addConstantDouble("kI", 0);
    private final Supplier<Double> kDAngle = anglePID.addConstantDouble("kD", 0);
    private final Supplier<Double> waitTimeAngle = anglePID.addConstantDouble("wait time", 0);
    private final Supplier<Double> toleranceAngle = anglePID.addConstantDouble("tolerance", 0);
    private final PIDSettings anglePIDSettings = new PIDSettings(kPAngle, kIAngle, kDAngle, waitTimeAngle,toleranceAngle);

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain(
                    new CANSparkMax(RobotMap.CAN.DRIVETRAIN_LEFT_SPARKMAX_1, CANSparkMaxLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.DRIVETRAIN_LEFT_SPARKMAX_2, CANSparkMaxLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.DRIVETRAIN_RIGHT_SPARKMAX_1, CANSparkMaxLowLevel.MotorType.kBrushless),
                    new CANSparkMax(RobotMap.CAN.DRIVETRAIN_RIGHT_SPARKMAX_2, CANSparkMaxLowLevel.MotorType.kBrushless));
        }
        return instance;
    }

    public Drivetrain(CANSparkMax left1, CANSparkMax left2, CANSparkMax right1, CANSparkMax right2) {
        super("drivetrain", left1, left2, right1, right2);
        this.leftEncoder = left1.getEncoder();
        this.rightEncoder = right1.getEncoder();
        configureDashboard();
    }

    public PIDSettings getDrivePIDSettings() {
        return drivePIDSettings;
    }

    public PIDSettings getAnglePIDSettings() {
        return anglePIDSettings;
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("right encoder", rightEncoder::getPosition);
        namespace.putNumber("left encoder", leftEncoder::getPosition);
    }
}
