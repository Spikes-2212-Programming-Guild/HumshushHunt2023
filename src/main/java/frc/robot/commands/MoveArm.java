package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerSubsystemTrapezically;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmFirstJoint;
import frc.robot.subsystems.ArmSecondJoint;

public class MoveArm extends ParallelCommandGroup {

    public enum ArmState {

        REST(0, 0), FLOOR_BACK(0, 0), FLOOR_FRONT(0, 0), DOUBLE_SUBSTATION(0, 0), BOTTOM(0, 0), CUBE_MID(0, 0),
        CUBE_TOP(0, 0), CONE_MID(0, 0), CONE_TOP(0, 0);

        public final double firstJointPosition;
        public final double secondJointPosition;

        ArmState(double firstJointPosition, double secondJointPosition) {
            this.firstJointPosition = firstJointPosition;
            this.secondJointPosition = secondJointPosition;
        }
    }

    public MoveArm(ArmFirstJoint firstJoint, ArmSecondJoint secondJoint, ArmState state) {
        super(new MoveSmartMotorControllerSubsystemTrapezically(firstJoint,
                        firstJoint.getPIDSettings(), firstJoint.getFeedForwardSettings(),
                        () -> state.firstJointPosition, firstJoint.getTrapezoidProfileSettings()),
                new MoveSmartMotorControllerSubsystemTrapezically(secondJoint,
                        secondJoint.getPIDSettings(), secondJoint.getFeedForwardSettings(),
                        () -> state.secondJointPosition, secondJoint.getTrapezoidProfileSettings()));
    }
}
