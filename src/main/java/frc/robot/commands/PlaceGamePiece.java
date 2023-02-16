package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.services.ArmGravityCompensation;
import frc.robot.subsystems.ArmFirstJoint;
import frc.robot.subsystems.ArmSecondJoint;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;

import java.util.function.Supplier;

public class PlaceGamePiece extends SequentialCommandGroup {

    public enum ArmState {
        FOLD_BELOW_180(90, 35, 0),
        FOLD_ABOVE_180(90, 325, 0),
        REST(0, 0, 0),
        FLOOR_BACK(80, 103, 0.5),
        FLOOR_FRONT(100, 255, 0),
        DOUBLE_SUBSTATION(0, 0, 0),
        BOTTOM(0, 0, 0),
        BACK_MID(49, 77, 0.5),
        BACK_TOP(3, 130, 0.7);

        public final double firstJointPosition;
        public final double secondJointPosition;
        public final double moveDuration;

        ArmState(double firstJointPosition, double secondJointPosition, double moveDuration) {
            this.firstJointPosition = firstJointPosition;
            this.secondJointPosition = secondJointPosition;
            this.moveDuration = moveDuration;
        }
    }

    private static final Supplier<Double> WAIT_TIME = () ->  0.1;

    public PlaceGamePiece(ArmFirstJoint firstJoint, ArmSecondJoint secondJoint, ArmState state) {
        addCommands(
                new InstantCommand(() -> Drivetrain.getInstance().setMode(CANSparkMax.IdleMode.kBrake)),
                new MoveSecondJoint(secondJoint, () -> ArmState.FOLD_BELOW_180.secondJointPosition, WAIT_TIME,
                        () -> state.moveDuration),
                new MoveFirstJoint(firstJoint, () -> state.firstJointPosition, WAIT_TIME,
                        () -> state.moveDuration),
                new MoveSecondJoint(secondJoint, () -> state.secondJointPosition, WAIT_TIME,
                        () -> state.moveDuration),
                new WaitCommand(0.7),
                new OpenGripper(Gripper.getInstance()),
                new WaitCommand(0.5),
                new MoveSecondJoint(secondJoint, () -> ArmState.FOLD_BELOW_180.secondJointPosition, WAIT_TIME,
                        () -> state.moveDuration),
                new CloseGripper(Gripper.getInstance()),
                new MoveFirstJoint(firstJoint, () -> 90.0, WAIT_TIME, () -> state.moveDuration),
                new InstantCommand(() -> Drivetrain.getInstance().setMode(CANSparkMax.IdleMode.kCoast)),
                new KeepArmStable(firstJoint, secondJoint, ArmGravityCompensation.getInstance())
        );
    }
}
