package frc.robot.commands;

import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.services.ArmGravityCompensation;
import frc.robot.subsystems.*;

import java.util.function.Supplier;

public class PlaceGamePiece extends SequentialCommandGroup {

    private static final Supplier<Double> WAIT_TIME = () -> 0.1;
    private static final double WAIT_PERIOD = 0.5;

    public PlaceGamePiece(ArmFirstJoint firstJoint, ArmSecondJoint secondJoint, ArmState state) {
        addRequirements(firstJoint, secondJoint, FakeArm.getInstance());
        if (state == ArmState.BACK_MID || state == ArmState.BACK_TOP || state == ArmState.FLOOR_BACK) {
            addCommands(
                    new InstantCommand(() -> Drivetrain.getInstance().setMode(CANSparkMax.IdleMode.kBrake)),
                    new MoveSecondJoint(secondJoint, () -> ArmState.FOLD_BELOW_180.secondJointPosition, WAIT_TIME,
                            () -> state.moveDuration),
                    new MoveFirstJoint(firstJoint, () -> state.firstJointPosition, WAIT_TIME,
                            () -> state.moveDuration),
                    new MoveSecondJoint(secondJoint, () -> state.secondJointPosition, WAIT_TIME,
                            () -> state.moveDuration),
//                new WaitCommand(2 * WAIT_PERIOD),
//                    new OpenGripper(Gripper.getInstance()),
//                    new WaitCommand(WAIT_PERIOD),
//                    new MoveSecondJoint(secondJoint, () -> ArmState.FOLD_BELOW_180.secondJointPosition, WAIT_TIME,
//                            () -> state.moveDuration),
//                    new CloseGripper(Gripper.getInstance()),
//                    new MoveFirstJoint(firstJoint, () -> 90.0, WAIT_TIME, () -> state.moveDuration),
                    new InstantCommand(() -> Drivetrain.getInstance().setMode(CANSparkMax.IdleMode.kCoast)),
                    new KeepArmStable(firstJoint, secondJoint, ArmGravityCompensation.getInstance())
            );
        } else {
            if (state == ArmState.FRONT_MID || state == ArmState.FRONT_TOP || state == ArmState.FLOOR_FRONT) {
                addCommands(
                        new InstantCommand(() -> Drivetrain.getInstance().setMode(CANSparkMax.IdleMode.kBrake)),
                        new MoveSecondJoint(secondJoint, () -> ArmState.FOLD_ABOVE_180.secondJointPosition, WAIT_TIME,
                                () -> state.moveDuration),
                        new MoveFirstJoint(firstJoint, () -> state.firstJointPosition, WAIT_TIME,
                                () -> state.moveDuration),
                        new MoveSecondJoint(secondJoint, () -> state.secondJointPosition, WAIT_TIME,
                                () -> state.moveDuration),
//                new WaitCommand(2 * WAIT_PERIOD),
//                        new OpenGripper(Gripper.getInstance()),
//                        new WaitCommand(WAIT_PERIOD),
//                        new MoveSecondJoint(secondJoint, () -> ArmState.FOLD_ABOVE_180.secondJointPosition, WAIT_TIME,
//                                () -> state.moveDuration),
//                        new CloseGripper(Gripper.getInstance()),
//                        new MoveFirstJoint(firstJoint, () -> 90.0, WAIT_TIME, () -> state.moveDuration),
                        new InstantCommand(() -> Drivetrain.getInstance().setMode(CANSparkMax.IdleMode.kCoast)),
                        new KeepArmStable(firstJoint, secondJoint, ArmGravityCompensation.getInstance())
                );
            }
        }
    }

    public enum ArmState {

        FOLD_BELOW_180(90, 35, 0),
        FOLD_ABOVE_180(90, 325, 0),
        REST(0, 0, 0),
        FLOOR_BACK(63, 120, 0.5),
        FLOOR_FRONT(100, 255, 0),
        DOUBLE_SUBSTATION(0, 0, 0),
        BOTTOM(0, 0, 0),
        BACK_MID(31.5, 75, 0.5),
        BACK_TOP(6.2, 130, 0.7),
        FRONT_MID(125, 282, 0.5),
        FRONT_TOP(156, 220, 0.7);

        public final double firstJointPosition;
        public final double secondJointPosition;
        public final double moveDuration;

        ArmState(double firstJointPosition, double secondJointPosition, double moveDuration) {
            this.firstJointPosition = firstJointPosition;
            this.secondJointPosition = secondJointPosition;
            this.moveDuration = moveDuration;
        }
    }
}
