package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerSubsystemTrapezically;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmFirstJoint;
import frc.robot.subsystems.ArmSecondJoint;

public class MoveArm extends ParallelCommandGroup {

    public MoveArm(ArmFirstJoint firstJoint, ArmSecondJoint secondJoint, ArmState state) {
        super(new MoveSmartMotorControllerSubsystemTrapezically(firstJoint,
                        firstJoint.getPIDSettings(), firstJoint.getFeedForwardSettings(),
                        () -> state.firstJointPosition, firstJoint.getTrapezoidProfileSettings()),
                new MoveSmartMotorControllerSubsystemTrapezically(secondJoint,
                        secondJoint.getPIDSettings(), secondJoint.getFeedForwardSettings(),
                        () -> state.secondJointPosition, secondJoint.getTrapezoidProfileSettings()));
    }

    public enum ArmState {

        RESTING(0, 0), COLLECTING_FLOOR(0, 0), DOUBLE_SUBSTATION(0, 0), PLACING_LOW(0, 0), PLACING_MID_CUBE(0, 0),
        PLACING_HIGH_CUBE(0, 0), PLACING_MID_CONE(0, 0), PLACING_HIGH_CONE(0, 0);

        public final double firstJointPosition;
        public final double secondJointPosition;

        ArmState(double firstJointPosition, double secondJointPosition) {
            this.firstJointPosition = firstJointPosition;
            this.secondJointPosition = secondJointPosition;
        }
    }
}


