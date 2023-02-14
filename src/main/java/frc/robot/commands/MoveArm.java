package frc.robot.commands;

import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerSubsystemTrapezically;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmFirstJoint;
import frc.robot.subsystems.ArmSecondJoint;

public class MoveArm extends SequentialCommandGroup {

    public enum ArmState {
        FOLD_BELOW_180(90, 35), FOLD_ABOVE_180(90, 325),
        REST(0, 0), FLOOR_BACK(0, 0), FLOOR_FRONT(0, 0), DOUBLE_SUBSTATION(0, 0), BOTTOM(0, 0), CUBE_MID(0, 0),
        CUBE_TOP(0, 0), CONE_MID(0, 0), CONE_TOP(6, 134);

        public final double firstJointPosition;
        public final double secondJointPosition;

        ArmState(double firstJointPosition, double secondJointPosition) {
            this.firstJointPosition = firstJointPosition;
            this.secondJointPosition = secondJointPosition;
        }
    }

    public MoveArm(ArmFirstJoint firstJoint, ArmSecondJoint secondJoint, ArmState state) {
        boolean folded = false;
        boolean firstReached = false;

        if (secondJoint.getAbsolutePosition() < 180) {
            addCommands(
                    new MoveSmartMotorControllerGenericSubsystem(secondJoint, secondJoint.getPIDSettings(),
                            secondJoint.getFeedForwardSettings(), UnifiedControlMode.POSITION,
                            () -> ArmState.FOLD_BELOW_180.secondJointPosition) {
                        @Override
                        public boolean isFinished() {
                            return secondJoint.getAbsolutePosition() <= ArmState.FOLD_BELOW_180.secondJointPosition;
                        }

                        @Override
                        public void end(boolean interrupted) {}
                    });
        } else {
            addCommands(new MoveSmartMotorControllerGenericSubsystem(
                    secondJoint, secondJoint.getPIDSettings(),
                    secondJoint.getFeedForwardSettings(), UnifiedControlMode.POSITION,
                    () -> ArmState.FOLD_ABOVE_180.secondJointPosition) {
                @Override
                public boolean isFinished() {
                    return secondJoint.getAbsolutePosition() >= ArmState.FOLD_ABOVE_180.secondJointPosition;
                }

                @Override
                public void end(boolean interrupted) {}
            });
        }

        ParallelRaceGroup raceGroup1 = new ParallelRaceGroup();
        raceGroup1.addCommands(
                new MoveSmartMotorControllerGenericSubsystem(firstJoint, firstJoint.getPIDSettings(),
                        firstJoint.getFeedForwardSettings(), UnifiedControlMode.POSITION,
                        () -> state.firstJointPosition) {
                    @Override
                    public boolean isFinished() {
                        return firstJoint.onTarget(UnifiedControlMode.POSITION, 1, setpoint.get());
                    }

                    @Override
                    public void end(boolean interrupted) {}
                }
        );
        if (secondJoint.getAbsolutePosition() < 180) {
            raceGroup1.addCommands(
                    new MoveSmartMotorControllerGenericSubsystem(secondJoint, secondJoint.getPIDSettings(),
                            secondJoint.getFeedForwardSettings(), UnifiedControlMode.POSITION,
                            () -> ArmState.FOLD_BELOW_180.secondJointPosition) {
                        @Override
                        public boolean isFinished() {
                            return false;
                        }
                    });
        } else {
            raceGroup1.addCommands(new MoveSmartMotorControllerGenericSubsystem(
                    secondJoint, secondJoint.getPIDSettings(),
                    secondJoint.getFeedForwardSettings(), UnifiedControlMode.POSITION,
                    () -> ArmState.FOLD_ABOVE_180.secondJointPosition) {
                @Override
                public boolean isFinished() {
                    return false;
                }
            });
        }

        addCommands(raceGroup1);

        ParallelCommandGroup parallelGroup = new ParallelCommandGroup(
                new MoveSmartMotorControllerGenericSubsystem(secondJoint,
                        secondJoint.getPIDSettings(), secondJoint.getFeedForwardSettings(), UnifiedControlMode.POSITION,
                        () -> state.secondJointPosition) {
                    @Override
                    public boolean isFinished() {
                        return false;
                    }
                },
                new MoveSmartMotorControllerGenericSubsystem(firstJoint, firstJoint.getPIDSettings(),
                        firstJoint.getFeedForwardSettings(), UnifiedControlMode.POSITION, () -> state.firstJointPosition) {
                    @Override
                    public boolean isFinished() {
                        return false;
                    }
                }
        );
        addCommands(parallelGroup);
    }
}
