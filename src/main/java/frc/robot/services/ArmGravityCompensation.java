package frc.robot.services;

import com.spikes2212.dashboard.RootNamespace;
import frc.robot.subsystems.ArmFirstJoint;
import frc.robot.subsystems.ArmSecondJoint;

import java.util.function.Supplier;

import static frc.robot.subsystems.ArmFirstJoint.GEAR_RATIO_ABSOLUTE_ENCODER_TO_ARM;
import static frc.robot.subsystems.ArmFirstJoint.GEAR_RATIO_MOTOR_TO_ABSOLUTE_ENCODER;

public class ArmGravityCompensation {

    private static final double GRAVITY = 9.81;
    private static final double STALL_TORQUE_TO_VOLTAGE = 4 / 1.41;
    private static final int FIRST_JOINT_MOTORS = 2;

    private final RootNamespace rootNamespace = new RootNamespace("arm gravity compensation");
    public final Supplier<Double> lm1 = rootNamespace.addConstantDouble("lm1", 2.5);
    public final Supplier<Double> l2 = rootNamespace.addConstantDouble("l2", 0.3);
    public final Supplier<Double> m2 = rootNamespace.addConstantDouble("m2", 4);
    public final Supplier<Double> lA = rootNamespace.addConstantDouble("la", 0.9);

    private final ArmFirstJoint firstJoint;
    private final ArmSecondJoint secondJoint;

    private static ArmGravityCompensation instance;

    public static ArmGravityCompensation getInstance() {
        if (instance == null) {
            instance = new ArmGravityCompensation(ArmFirstJoint.getInstance(), ArmSecondJoint.getInstance());
        }
        return instance;
    }

    private ArmGravityCompensation(ArmFirstJoint firstJoint, ArmSecondJoint secondJoint) {
        this.firstJoint = firstJoint;
        this.secondJoint = secondJoint;
    }

    public double configureFirstJointG(double firstJointAngle, double secondJointAngle) {
        double torque = lm1.get() * GRAVITY * Math.cos(Math.toRadians(firstJointAngle))
                + (l2.get() * Math.cos(Math.toRadians(firstJointAngle + secondJointAngle)) +
                lA.get() * Math.cos(Math.toRadians(firstJointAngle))) * m2.get() * GRAVITY;

        double stallTorque = torque * (GEAR_RATIO_ABSOLUTE_ENCODER_TO_ARM * GEAR_RATIO_MOTOR_TO_ABSOLUTE_ENCODER);
        double voltage = stallTorque * STALL_TORQUE_TO_VOLTAGE / FIRST_JOINT_MOTORS
                * firstJoint.getFeedForwardSettings().getkG();
        firstJoint.setArbitraryFeedForward(voltage);
        return voltage;
    }

    public double configureSecondJointG(double firstJointAngle, double secondJointAngle) {
        double voltage = secondJoint.getFeedForwardSettings().getkG() *
                Math.cos(Math.toRadians(firstJointAngle + secondJointAngle));
        secondJoint.setArbitraryFeedForward(voltage);
        return voltage;
    }

    public void zeroGs() {
        firstJoint.setArbitraryFeedForward(0);
        secondJoint.setArbitraryFeedForward(0);
    }
}
