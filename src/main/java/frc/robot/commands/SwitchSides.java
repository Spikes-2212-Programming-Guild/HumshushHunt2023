package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.services.ArmGravityCompensation;
import frc.robot.subsystems.ArmFirstJoint;
import frc.robot.subsystems.ArmSecondJoint;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;

import java.util.function.Supplier;

public class SwitchSides extends SequentialCommandGroup {

    private static final Supplier<Double> WAIT_TIME = () -> 0.005;
    private static final Supplier<Double> MOVE_DURATION = () -> 0.5;

    private final ArmFirstJoint firstJoint;
    private final ArmSecondJoint secondJoint;
    private final Gripper gripper;

//    @Override
//    public void initialize() {
//        CommandBase switchSides;
//        if (secondJoint.isBack()) {
//            switchSides = moveArmFromBack(firstJoint, secondJoint, gripper);
//        } else {
//            switchSides = moveArmFromFront(firstJoint, secondJoint, gripper);
//        }
//        switchSides.schedule();
//    }

    public SwitchSides(ArmFirstJoint firstJoint, ArmSecondJoint secondJoint, Gripper gripper, boolean isBack) {
        this.firstJoint = firstJoint;
        this.secondJoint = secondJoint;
        this.gripper = gripper;
        if (isBack) {
            addCommands(
                    new InstantCommand(() -> Drivetrain.getInstance().setMode(CANSparkMax.IdleMode.kBrake)),
                    new CloseGripper(gripper),
                    new MoveSecondJoint(secondJoint, () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.secondJointPosition,
                            WAIT_TIME, MOVE_DURATION),
                    new MoveFirstJoint(firstJoint, ()
                            -> 180.0, WAIT_TIME, MOVE_DURATION),
                    new MoveSecondJoint(secondJoint, () -> 330.0, WAIT_TIME, () -> 1.2),
//                    new ParallelCommandGroup(
//                            new MoveSecondJoint(secondJoint, () -> 325.0, WAIT_TIME, () -> 0.8),
                    new MoveFirstJoint(firstJoint, () -> 90.0, WAIT_TIME, () -> 0.8),
                    new InstantCommand(() -> Drivetrain.getInstance().setMode(CANSparkMax.IdleMode.kCoast)),
//                    ),
                    new KeepArmStable(firstJoint, secondJoint, ArmGravityCompensation.getInstance())
            );
        } else {
            addCommands(
                    new InstantCommand(() -> Drivetrain.getInstance().setMode(CANSparkMax.IdleMode.kBrake)),
                    new CloseGripper(gripper),
                    new MoveSecondJoint(secondJoint, () -> PlaceGamePiece.ArmState.FOLD_ABOVE_180.secondJointPosition,
                            WAIT_TIME, MOVE_DURATION),
                    new MoveFirstJoint(firstJoint, () -> -5.0, WAIT_TIME, MOVE_DURATION),
//                    new MoveSecondJoint(secondJoint, () -> 180.0, WAIT_TIME, MOVE_DURATION),
//                    new ParallelCommandGroup(
                    new MoveSecondJoint(secondJoint, () -> 35.0, WAIT_TIME, () -> 1.2),
                    new MoveFirstJoint(firstJoint, () -> 90.0, WAIT_TIME, () -> 0.8),
//                    ),
                    new InstantCommand(() -> Drivetrain.getInstance().setMode(CANSparkMax.IdleMode.kCoast)),
                    new KeepArmStable(firstJoint, secondJoint, ArmGravityCompensation.getInstance())
            );
        }
    }

//    private SequentialCommandGroup moveArmFromBack(ArmFirstJoint firstJoint, ArmSecondJoint secondJoint, Gripper gripper) {
//        return new SequentialCommandGroup(
////                new InstantCommand(() -> Drivetrain.getInstance().setMode(CANSparkMax.IdleMode.kBrake)),
//                new CloseGripper(gripper),
//                new MoveSecondJoint(secondJoint, () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.secondJointPosition,
//                        WAIT_TIME, MOVE_DURATION),
//                new MoveFirstJoint(firstJoint, ()
//                        -> 180.0, WAIT_TIME, MOVE_DURATION),
//                new MoveSecondJoint(secondJoint, () -> 330.0, WAIT_TIME, () -> 1.2),
//                new MoveFirstJoint(firstJoint, () -> 90.0, WAIT_TIME, () -> 0.8),
////                new InstantCommand(() -> Drivetrain.getInstance().setMode(CANSparkMax.IdleMode.kCoast)),
//                new KeepArmStable(firstJoint, secondJoint, ArmGravityCompensation.getInstance())
//        );
//    }
//
//    private SequentialCommandGroup moveArmFromFront(ArmFirstJoint firstJoint, ArmSecondJoint secondJoint, Gripper gripper) {
//        return new SequentialCommandGroup(
////                new InstantCommand(() -> Drivetrain.getInstance().setMode(CANSparkMax.IdleMode.kBrake)),
//                new CloseGripper(gripper),
//                new MoveSecondJoint(secondJoint, () -> PlaceGamePiece.ArmState.FOLD_ABOVE_180.secondJointPosition,
//                        WAIT_TIME, MOVE_DURATION),
//                new MoveFirstJoint(firstJoint, () -> -5.0, WAIT_TIME, MOVE_DURATION),
//                new MoveSecondJoint(secondJoint, () -> 35.0, WAIT_TIME, () -> 1.2),
//                new MoveFirstJoint(firstJoint, () -> 90.0, WAIT_TIME, () -> 0.8),
////                new InstantCommand(() -> Drivetrain.getInstance().setMode(CANSparkMax.IdleMode.kCoast)),
//                new KeepArmStable(firstJoint, secondJoint, ArmGravityCompensation.getInstance())
//        );
//    }
}
