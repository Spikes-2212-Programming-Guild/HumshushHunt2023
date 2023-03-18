package frc.robot.commands.autonomous;

import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.commands.PlaceGamePiece.ArmState;
import frc.robot.services.ArmGravityCompensation;
import frc.robot.subsystems.ArmFirstJoint;
import frc.robot.subsystems.ArmSecondJoint;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;

import java.util.function.Supplier;

public class ClimbWithExit extends SequentialCommandGroup {

    private static final RootNamespace ROOT = new RootNamespace("climb with exit");
    //done via trigonometry
    private static final Supplier<Double> RAMP_DISTANCE = ROOT.addConstantDouble("ramp distance", 207.641);
    //measured in pathplanner
    private static final Supplier<Double> DISTANCE_TO_RAMP = ROOT.addConstantDouble("distance to ramp", 135);
    //robot length with bumpers
    private static final Supplier<Double> DISTANCE_AFTER_RAMP
            = ROOT.addConstantDouble("distance after ramp", 0.95);
    private static final double POST_OPEN_GRIPPER_WAIT_TIME = 0.06;
    private static final Supplier<Double> MIN_WAIT_TIME = () -> 0.005;
    private static final Supplier<Double> FOLD_SECOND_JOINT_TIMEOUT = () -> ArmState.FOLD_BELOW_180.moveDuration - 0.2;

    public ClimbWithExit(Drivetrain drivetrain, ArmFirstJoint firstJoint, ArmSecondJoint secondJoint, Gripper gripper) {
        addCommands(
                new PlaceGamePiece(ArmFirstJoint.getInstance(), ArmSecondJoint.getInstance(),
                        ArmState.BACK_TOP),
                new OpenGripper(Gripper.getInstance()),
                new WaitCommand(POST_OPEN_GRIPPER_WAIT_TIME),
                new ParallelRaceGroup(
                        new KeepFirstJointStable(firstJoint, secondJoint, ArmGravityCompensation.getInstance()),
                        new MoveSecondJoint(ArmSecondJoint.getInstance(),
                                () -> ArmState.FOLD_BELOW_180.secondJointPosition, MIN_WAIT_TIME,
                                FOLD_SECOND_JOINT_TIMEOUT)
                ),
                new CloseGripper(Gripper.getInstance())
        );
    }
}
