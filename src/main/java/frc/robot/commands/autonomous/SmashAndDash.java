package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.spikes2212.command.drivetrains.commands.DriveTank;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.services.ArmGravityCompensation;
import frc.robot.services.VisionService;
import frc.robot.subsystems.ArmFirstJoint;
import frc.robot.subsystems.ArmSecondJoint;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class SmashAndDash extends BasePathAuto {

    private static final double MAX_VELOCITY = 2.5;
    private static final double MAX_ACCELERATION = 2;

    public SmashAndDash(Drivetrain drivetrain) {
        super(drivetrain, getEventMap());
//        super(drivetrain, new HashMap<>());
    }

    public CommandBase getCommand() {
        List<PathPlannerTrajectory> trajectory = PathPlanner.loadPathGroup("Smash And Dash",
                new PathConstraints(MAX_VELOCITY, MAX_ACCELERATION));
        System.out.println(trajectory.get(0).toString());
//        return new InstantCommand(() -> drivetrain.configureLoop(drivetrain.getLeftPIDSettings(),
//                drivetrain.getRightPIDSettings(), drivetrain.getFeedForwardSettings())).andThen(fullAuto(trajectory));
        return fullAuto(trajectory);
    }

    private static Map<String, Command> getEventMap() {
        Drivetrain drivetrain = Drivetrain.getInstance();
        VisionService vision = VisionService.getInstance();
        ArmFirstJoint firstJoint = ArmFirstJoint.getInstance();
        ArmSecondJoint secondJoint = ArmSecondJoint.getInstance();
        ArmGravityCompensation compensation = ArmGravityCompensation.getInstance();
        Gripper gripper = Gripper.getInstance();
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("putGP", new SequentialCommandGroup(
                new PrintCommand("put gp"),
                new PlaceGamePiece(ArmFirstJoint.getInstance(), ArmSecondJoint.getInstance(),
                        PlaceGamePiece.ArmState.BACK_TOP),
                new OpenGripper(Gripper.getInstance()),
                new MoveSecondJoint(ArmSecondJoint.getInstance(),
                        () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.secondJointPosition, () -> 0.005,
                        () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.moveDuration + 0.2),
                new CloseGripper(Gripper.getInstance()),
                new MoveFirstJoint(ArmFirstJoint.getInstance(), () -> 110.0, () -> 0.005,
                        () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.moveDuration + 0.2)
        ));
        eventMap.put("takeGP",
                new SequentialCommandGroup(
//                        new SwitchSides(firstJoint, secondJoint, gripper, true),
                        new PlaceGamePiece(firstJoint, secondJoint, PlaceGamePiece.ArmState.FLOOR_FRONT),
                        new OpenGripper(gripper),
                        new WaitCommand(3),
                        new CloseGripper(gripper)
                ));
        eventMap.put("putGP2", new SequentialCommandGroup(
                new WaitCommand(1),
                new MoveFirstJoint(firstJoint, () -> PlaceGamePiece.ArmState.BACK_TOP.firstJointPosition, () -> 0.005,
                        () -> PlaceGamePiece.ArmState.BACK_TOP.moveDuration),
                new ParallelRaceGroup(
                        new KeepFirstJointStable(firstJoint, secondJoint, ArmGravityCompensation.getInstance()),
                        new MoveSecondJoint(secondJoint, () -> PlaceGamePiece.ArmState.BACK_TOP.secondJointPosition, () -> 0.005,
                                () -> PlaceGamePiece.ArmState.BACK_TOP.moveDuration)),
                new DriveTank(drivetrain, () -> -0.5, () -> -0.5).withTimeout(0.5),
//                new CenterWithLimelight(drivetrain, vision, VisionService.LimelightPipeline.APRIL_TAG) {
//                    @Override
//                    public void initialize() {
//                        super.initialize();
//                        moveValue = () -> -0.5;
//                    }
//                }
                new OpenGripper(gripper)
        ));
        eventMap.put("switchSides1", new SwitchSides(firstJoint, secondJoint, gripper, true));
        eventMap.put("switchSides2", new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new MoveSecondJoint(secondJoint, () -> PlaceGamePiece.ArmState.FOLD_ABOVE_180.secondJointPosition,
                                () -> 0.005, () -> 0.7),
                        new MoveFirstJoint(firstJoint, () -> 5.0, () -> 0.005, () -> 0.7)),
                new MoveFirstJoint(firstJoint, () -> PlaceGamePiece.ArmState.BACK_TOP.firstJointPosition,
                        () -> 0.005, () -> 0.7),
                new MoveSecondJoint(secondJoint, () -> PlaceGamePiece.ArmState.BACK_TOP.secondJointPosition,
                        () -> 0.005, () -> 0.7))
        );
        return eventMap;
    }
}
