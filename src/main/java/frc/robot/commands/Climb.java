package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.spikes2212.dashboard.Namespace;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;

//@TODO other direction
public class Climb extends CommandBase {

    private static final RootNamespace namespace = new RootNamespace("climb");
    private static final Namespace testing = namespace.addChild("testing");
    private static final Supplier<Double> PRE_CLIMB_SPEED = namespace.addConstantDouble("pre climb speed", 0.6);
    private static final Supplier<Double> MID_CLIMB_SPEED = namespace.addConstantDouble("mid climb speed", 0.3);
    private static final Supplier<Double> ROTATE_SPEED = namespace.addConstantDouble("rotate speed climb speed", 0.2);
    private static final Supplier<Double> MIN_MID_CLIMB_VOLTAGE = namespace.addConstantDouble("minimum mid climb voltage", 2.4);
    private static final Supplier<Double> PRE_CLIMB_TOLERANCE = namespace.addConstantDouble("pre climb tolerance", 15);
    private static final Supplier<Double> MID_CLIMB_TOLERANCE = namespace.addConstantDouble("mid climb tolerance", 3);
    private static final Supplier<Double> ENCODERS_SETPOINT = namespace.addConstantDouble("encoders setpoint", 1);
    private static final Supplier<Double> YAW_SETPOINT = namespace.addConstantDouble("yaw setpoint", 75);
    private static final Supplier<Double> WAIT_TIME = namespace.addConstantDouble("wait time", 0.5);

    private final Drivetrain drivetrain;

    private boolean startedClimbing;
    private boolean doneBrake;
    private double lastTimeNotOnTarget;
    private boolean startedRotating;

    public Climb(Drivetrain drivetrain) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        drivetrain.resetGyro();
        lastTimeNotOnTarget = Timer.getFPGATimestamp();
        doneBrake = false;
        startedClimbing = false;
        startedRotating = false;
        drivetrain.setMode(CANSparkMax.IdleMode.kCoast);
    }

    @Override
    public void execute() {
        double pitch = drivetrain.getPitch();
        namespace.putBoolean("pitch smaller than tolerance", Math.abs(pitch) <= MID_CLIMB_TOLERANCE.get());
        if (!startedClimbing) {
            drivetrain.arcadeDrive(PRE_CLIMB_SPEED.get(), 0);
            startedClimbing = Math.abs(pitch) > PRE_CLIMB_TOLERANCE.get();
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
            if (startedClimbing) drivetrain.resetEncoders();
        } else {
            if (Math.abs(pitch) <= MID_CLIMB_TOLERANCE.get()) {
                if (!doneBrake) {
                    drivetrain.setMode(CANSparkMax.IdleMode.kBrake);
                    doneBrake = true;
                }
//                }
//                if (Math.abs(drivetrain.getYaw()) <= YAW_SETPOINT.get()) {
//                    startedRotating = true;
//                    drivetrain.arcadeDrive(0, ROTATE_SPEED.get());
//                    lastTimeNotOnTarget = Timer.getFPGATimestamp();
//                }
            } else {
                drivetrain.arcadeDrive(
                        Math.signum(pitch) * Math.max(
                                Math.abs(MID_CLIMB_SPEED.get() *
                                        Math.sqrt((ENCODERS_SETPOINT.get() - Math.abs(drivetrain.getLeftPosition())))),
                                Math.abs(MIN_MID_CLIMB_VOLTAGE.get() / RobotController.getBatteryVoltage())), 0);
                lastTimeNotOnTarget = Timer.getFPGATimestamp();
            }
        }

    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - lastTimeNotOnTarget >= WAIT_TIME.get();
//        return Math.abs(drivetrain.getPitch()) < MID_CLIMB_TOLERANCE.get() && startedClimbing;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
//        drivetrain.setMode(CANSparkMax.IdleMode.kCoast);
    }
}
//package frc.robot.commands;
//
//import com.spikes2212.dashboard.RootNamespace;
//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.subsystems.Drivetrain;
//
//import java.util.function.Supplier;
//
//public class Climb extends CommandBase {
//
//    private static final RootNamespace namespace = new RootNamespace("climb");
//    private static final Supplier<Double> preClimbSpeed = namespace.addConstantDouble("pre climb speed", 0.6);
//    private static final Supplier<Double> midClimbSpeed = namespace.addConstantDouble("mid speed climb", 0.3);
//    private static final Supplier<Double> preClimbTolerance = namespace.addConstantDouble("pre climb tolerance", 15);
//    private static final Supplier<Double> midClimbTolerance = namespace.addConstantDouble("mid climb tolerance", 3);
//    private static final Supplier<Double> yawSetpoint = namespace.addConstantDouble("yaw setpoint", 75);
//    private static final Supplier<Double> waitTime = namespace.addConstantDouble("wait time", 0.5);
//
//    private final Drivetrain drivetrain;
//
//    private boolean startedClimbing;
//    private double lastTimeNotOnTarget;
//
//    public Climb(Drivetrain drivetrain) {
//        addRequirements(drivetrain);
//        this.drivetrain = drivetrain;
//    }
//
//    @Override
//    public void initialize() {
//        startedClimbing = false;
//        lastTimeNotOnTarget = Timer.getFPGATimestamp();
//    }
//
//    @Override
//    public void execute() {
//        double pitch = drivetrain.getPitch();
//        if (!startedClimbing) {
//            drivetrain.arcadeDrive(preClimbSpeed.get(), 0);
//            startedClimbing = Math.abs(pitch) > preClimbTolerance.get();
//            lastTimeNotOnTarget = Timer.getFPGATimestamp();
//        } else {
//            if (Math.abs(pitch) <= midClimbTolerance.get()) {
////                if (Math.abs(drivetrain.getYaw()) <= yawSetpoint.get()) {
////                    drivetrain.arcadeDrive(0, midClimbSpeed.get());
//            } else {
//                drivetrain.arcadeDrive(Math.signum(pitch) * midClimbSpeed.get(), 0);
//                lastTimeNotOnTarget = Timer.getFPGATimestamp();
//            }
//        }
//    }
//
//    @Override
//    public boolean isFinished() {
//        return Timer.getFPGATimestamp() - lastTimeNotOnTarget >= waitTime.get();
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//        drivetrain.stop();
//    }
//}
