package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.services.ArmGravityCompensation;
import frc.robot.subsystems.ArmFirstJoint;
import frc.robot.subsystems.ArmSecondJoint;

public class MoveArmToFloor extends SequentialCommandGroup {

    public MoveArmToFloor(ArmFirstJoint firstJoint, ArmSecondJoint secondJoint, ArmGravityCompensation compensation,
                          PlaceGamePiece.ArmState state) {
        addCommands(
                new ParallelCommandGroup(
                        new MoveFirstJoint(firstJoint, () -> state.firstJointPosition, () -> 0.1, () -> 0.3),
                        new MoveSecondJoint(secondJoint, () -> state.secondJointPosition, () -> 0.1, () -> 0.3)
                ),
                new KeepArmStable(firstJoint, secondJoint, compensation)
        );
    }
}
