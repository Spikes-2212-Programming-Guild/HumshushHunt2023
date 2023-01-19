package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.spikes2212.command.drivetrains.TankDrivetrain;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.RobotMap;
import java.util.function.Supplier;

public class Drivetrain extends TankDrivetrain {

    private static Drivetrain instance;

    private final Supplier<Double> kPDrive = namespace.addConstantDouble("kP", 0);
    private final Supplier<Double> kIDrive = namespace.addConstantDouble("kI", 0);
    private final Supplier<Double> kDDrive = namespace.addConstantDouble("kD", 0);
    private final Supplier<Double> toleranceDrive = namespace.addConstantDouble("tolerance", 0);
    private final Supplier<Double> waitTimeDrive = namespace.addConstantDouble("wait time", 0);

    private final CANSparkMax left1;
    private final CANSparkMax left2;
    private final CANSparkMax right1;
    private final CANSparkMax right2;

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
        super("drivetrain", new MotorControllerGroup(left1, left2), new MotorControllerGroup(right1, right2));
        this.left1 = left1;
        this.left2 = left2;
        this.right1 = right1;
        this.right2 = right2;
        this.leftEncoder = left1.getEncoder();
        this.rightEncoder = right1.getEncoder();
        configureDashboard();
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("right encoder", rightEncoder::getPosition);
        namespace.putNumber("left encoder", leftEncoder::getPosition);
    }
}
