package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerSubsystemTrapezically;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmFirstJoint;
import frc.robot.subsystems.ArmSecondJoint;

public class MoveArm extends ParallelCommandGroup {

    public MoveArm(ArmFirstJoint firstJoint, ArmSecondJoint secondJoint, ArmState state) {
        super(new MoveSmartMotorControllerSubsystemTrapezically(firstJoint,
                        firstJoint.getPIDSettings(), firstJoint.getFeedForwardSettings(),
                        state::getFirstJointPosition, firstJoint.getTrapezoidProfileSettings()),
                new MoveSmartMotorControllerSubsystemTrapezically(secondJoint,
                        secondJoint.getPIDSettings(), secondJoint.getFeedForwardSettings(),
                        state::getSecondJointPosition, secondJoint.getTrapezoidProfileSettings()));
    }

    public enum ArmState {

        RESTING(0, 0), COLLECTING(0, 0), PLACING_LOW(0, 0), PLACING_MID_CUBE(0, 0), PLACING_HIGH_CUBE(0, 0),
        PLACING_MID_CONE(0, 0), PLACING_HIGH_CONE(0, 0);

        private final double firstJointPosition;
        private final double secondJointPosition;

        ArmState(double firstJointPosition, double secondJointPosition) {
            this.firstJointPosition = firstJointPosition;
            this.secondJointPosition = secondJointPosition;
        }

        public double getFirstJointPosition() {
            return this.firstJointPosition;
        }

        public double getSecondJointPosition() {
            return this.secondJointPosition;
        }
    }
}


