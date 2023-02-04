package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivotrain extends SubsystemBase {

    JoystickSim joystickSim;
    DifferentialDriveOdometry odometry;
    DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
            DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
            7.29,                    // 7.29:1 gearing reduction.
            7.5,                     // MOI of 7.5 kg m^2 (from CAD model).
            60.0,                    // The mass of the robot is 60 kg.
            Units.inchesToMeters(3), // The robot uses 3" radius wheels.
            0.7112,                  // The track width is 0.7112 meters.

            VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
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

    public Drivotrain() {
        joystickSim = new JoystickSim(0);
        m_leftEncoder.setDistancePerPulse(2 * Math.PI * 3 * 0.0254 / 4096);
        m_rightEncoder.setDistancePerPulse(2 * Math.PI * 3 * 0.0254 / 4096);
        SmartDashboard.putData("Field", m_field);
        odometry = new DifferentialDriveOdometry(new Rotation2d(), 0, 0);

    }

    public void periodic() {
        odometry.update(m_gyro.getRotation2d(),
                m_leftEncoder.getDistance(),
                m_rightEncoder.getDistance());
        m_field.setRobotPose(odometry.getPoseMeters());
    }

    public void setSpeeds(double leftSpeed, double rightSpeed) {
        m_leftMotor.set(leftSpeed);
        m_rightMotor.set(rightSpeed);
        m_leftMotor.set(leftSpeed);
        m_rightSlave.set(rightSpeed);
    }

    public void simulationPeriodic() {
        m_driveSim.setInputs(m_leftMotor.get() * RobotController.getInputVoltage(),
                m_rightMotor.get() * RobotController.getInputVoltage());

        m_driveSim.update(0.02);

        m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
        m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
        m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
        m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
        m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());
    }
}
