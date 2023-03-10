package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.spikes2212.command.drivetrains.commands.DriveArcade;
import com.spikes2212.command.drivetrains.commands.DriveTank;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.commands.PlaceGamePiece.ArmState;
import frc.robot.subsystems.*;
import frc.robot.services.*;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

public class SmashAndDash extends BasePathAuto {

    private static final RootNamespace ROOT = new RootNamespace("smash and dash testing");

    private static final double
            MAX_VELOCITY = 1;
    private static final double MAX_ACCELERATION = 1;
    private static final Supplier<Double> MIN_WAIT_TIME = () -> 0.005;
    private static final Supplier<Double> MOVE_VALUE_TO_CUBE = ROOT.addConstantDouble("move value to cube", 0.4);
    private static final Supplier<Double> ADDITIONAL_DISTANCE_TO_CUBE
            = ROOT.addConstantDouble("additional distance to cube", 0.0);

    private static final Supplier<Double> POST_PUT_GP_FIRST_JOINT_TARGET = () -> 110.0;
    private static final Supplier<Double> SECOND_JOINT_TAKE_CUBE_POSITION = () -> 240.0;
    private static final Supplier<Double> SECOND_JOINT_TAKE_CUBE_MOVE_DURATION = () -> 0.2;
    private static final Supplier<Double> DRIVE_TO_GRID_SPEED = () -> -0.5;
    private static final double DRIVE_TO_GRID_TIMEOUT = 0.5;
    private static final Supplier<Double> SWITCH_SIDES_GENERAL_MOVE_DURATION = () -> 0.5;
    private static final Supplier<Double> SWITCH_SIDES_LOW_MOVE_DURATION = () -> 0.2;
    private static final Supplier<Double> SWITCH_SIDES_1_FIRST_JOINT_TOP_POSITION = () -> 0.0;
    private static final Supplier<Double> SWITCH_SIDES_1_SECOND_JOINT_FOLD_POSITION = () -> 300.0;
    private static final Supplier<Double> SWITCH_SIDES_1_FIRST_JOINT_FLO0R_POSITION = () -> 77.0;
    private static final Supplier<Double> SWITCH_SIDES_1_SECOND_JOINT_FLO0R_POSITION = () -> 240.0;

    public SmashAndDash(Drivetrain drivetrain) {
        super(drivetrain, getEventMap());
        ROOT.putData("move to cube", moveToCube(drivetrain, VisionService.getInstance()));
//        super(drivetrain, new HashMap<>());
    }

    public CommandBase getCommand() {
        List<PathPlannerTrajectory> trajectory = PathPlanner.loadPathGroup("Smash And Dash",
                new PathConstraints(MAX_VELOCITY, MAX_ACCELERATION));
        System.out.println(trajectory.get(0).toString());
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
                        ArmState.BACK_TOP),
                new OpenGripper(Gripper.getInstance()),
                //fold second joint
                new MoveSecondJoint(ArmSecondJoint.getInstance(),
                        () -> ArmState.FOLD_BELOW_180.secondJointPosition, MIN_WAIT_TIME,
                        () -> ArmState.FOLD_BELOW_180.moveDuration + 0.2),
                new CloseGripper(Gripper.getInstance()),
                //
                new MoveFirstJoint(ArmFirstJoint.getInstance(), POST_PUT_GP_FIRST_JOINT_TARGET, MIN_WAIT_TIME,
                        () -> ArmState.FOLD_BELOW_180.moveDuration + 0.2)
        ));
        eventMap.put("takeGP",
                new SequentialCommandGroup(
                        new ConditionalCommand(
                                new MoveSecondJoint(secondJoint, SECOND_JOINT_TAKE_CUBE_POSITION, MIN_WAIT_TIME,
                                        SECOND_JOINT_TAKE_CUBE_MOVE_DURATION),
                                new InstantCommand(() -> {
                                }),
                                () -> !secondJoint.isBack()
                        ),
                        new ParallelRaceGroup(
                                new KeepSecondJointStable(firstJoint, secondJoint, compensation),
                                new SequentialCommandGroup(
                                        new OpenGripper(gripper),
//                                        new CenterOnGamePiece(drivetrain, vision, VisionService.PhotonVisionPipeline.CUBE).withTimeout(0.5),
                                        moveToCube(drivetrain, vision),
                                        new CloseGripper(gripper)
                                )
                        )
                )
        );
        eventMap.put("putGP2", new SequentialCommandGroup(
                new MoveFirstJoint(firstJoint, () -> ArmState.BACK_TOP.firstJointPosition, MIN_WAIT_TIME,
                        () -> ArmState.BACK_TOP.moveDuration),
                new ParallelRaceGroup(
                        new KeepFirstJointStable(firstJoint, secondJoint, ArmGravityCompensation.getInstance()),
                        new MoveSecondJoint(secondJoint, () -> ArmState.BACK_TOP.secondJointPosition, MIN_WAIT_TIME,
                                () -> ArmState.BACK_TOP.moveDuration)),
                new DriveTank(drivetrain, DRIVE_TO_GRID_SPEED, DRIVE_TO_GRID_SPEED).withTimeout(DRIVE_TO_GRID_TIMEOUT),
//                new CenterWithLimelight(drivetrain, vision, VisionService.LimelightPipeline.APRIL_TAG) {
//                    @Override
//                    public void initialize() {
//                        super.initialize();
//                        moveValue = () -> -0.5;
//                    }
//                }
                new OpenGripper(gripper)
        ));
        eventMap.put("switchSides1",
                new SequentialCommandGroup(
                        new MoveSecondJoint(secondJoint,
                                () -> PlaceGamePiece.ArmState.FOLD_BELOW_180.secondJointPosition, MIN_WAIT_TIME,
                                SWITCH_SIDES_GENERAL_MOVE_DURATION),
                        new MoveFirstJoint(firstJoint, SWITCH_SIDES_1_FIRST_JOINT_TOP_POSITION, MIN_WAIT_TIME,
                                SWITCH_SIDES_GENERAL_MOVE_DURATION),
                        new MoveSecondJoint(secondJoint, SWITCH_SIDES_1_SECOND_JOINT_FOLD_POSITION, MIN_WAIT_TIME,
                                SWITCH_SIDES_GENERAL_MOVE_DURATION),
                        new ParallelRaceGroup(
                                new MoveFirstJoint(firstJoint, SWITCH_SIDES_1_FIRST_JOINT_FLO0R_POSITION,
                                        MIN_WAIT_TIME, SWITCH_SIDES_GENERAL_MOVE_DURATION),
                                new KeepSecondJointStable(firstJoint, secondJoint, compensation)
                        ),
                        new MoveSecondJoint(secondJoint, SWITCH_SIDES_1_SECOND_JOINT_FLO0R_POSITION, MIN_WAIT_TIME,
                                SWITCH_SIDES_LOW_MOVE_DURATION),
                        new KeepSecondJointStable(firstJoint, secondJoint, compensation)
                )
        );
        eventMap.put("switchSides2", new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new MoveSecondJoint(secondJoint, () -> ArmState.FOLD_ABOVE_180.secondJointPosition,
                                () -> 0.005, () -> 0.7),
                        new MoveFirstJoint(firstJoint, () -> 5.0, () -> 0.005, () -> 0.7)),
                new MoveFirstJoint(firstJoint, () -> ArmState.BACK_TOP.firstJointPosition,
                        () -> 0.005, () -> 0.7),
                new MoveSecondJoint(secondJoint, () -> ArmState.BACK_TOP.secondJointPosition,
                        () -> 0.005, () -> 0.7))
        );
        return eventMap;
    }

    private static CommandBase moveToCube(Drivetrain drivetrain, VisionService visionService) {
        return new DriveArcade(drivetrain, MOVE_VALUE_TO_CUBE, () -> 0.0) {
            private double initialPos;
            private double distanceFromCube;

            @Override
            public void initialize() {
                initialPos = drivetrain.getLeftPosition();
                distanceFromCube = 0.15;
//                distanceFromCube = visionService.getPhotonVisionDistanceFromTarget();
            }

            @Override
            public boolean isFinished() {
                return Math.abs(drivetrain.getLeftPosition() - initialPos)
                        >= distanceFromCube / 100 + ADDITIONAL_DISTANCE_TO_CUBE.get();
            }
        };
    }

    public static void update() {
        ROOT.update();
    }
}
