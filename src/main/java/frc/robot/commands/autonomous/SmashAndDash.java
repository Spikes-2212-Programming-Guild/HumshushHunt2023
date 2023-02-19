package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.services.ArmGravityCompensation;
import frc.robot.services.VisionService;
import frc.robot.subsystems.ArmFirstJoint;
import frc.robot.subsystems.ArmSecondJoint;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;

import java.util.HashMap;
import java.util.Map;

public class SmashAndDash extends BasePathAuto {

    private static final double MAX_VELOCITY = 1.5;
    private static final double MAX_ACCELERATION = 1.5;

    public SmashAndDash(Drivetrain drivetrain) {
        super(drivetrain, getEventMap());
    }

    public CommandBase getCommand() {
        return fullAuto(PathPlanner.loadPath("Smash And Dash",
                new PathConstraints(MAX_VELOCITY, MAX_ACCELERATION)));
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
                new CenterWithLimelight(drivetrain, vision, VisionService.LimelightPipeline.APRIL_TAG).withTimeout(1),
                new PlaceGamePiece(firstJoint, secondJoint, PlaceGamePiece.ArmState.BACK_TOP).withTimeout(5)
        ));
        eventMap.put("takeGP", new SequentialCommandGroup(
                new OpenGripper(gripper),
                new WaitCommand(10),
                new CloseGripper(gripper)
        ));
        eventMap.put("putGP2", new SequentialCommandGroup(
                new CenterWithLimelight(drivetrain, vision, VisionService.LimelightPipeline.HIGH_RRT).withTimeout(1),
                new PlaceGamePiece(firstJoint, secondJoint, PlaceGamePiece.ArmState.BACK_TOP)
        ));
        eventMap.put("switchSides", new SequentialCommandGroup(
                new InstantCommand(() -> {
                    if (secondJoint.isBack()) {
                        new SwitchSides(firstJoint, secondJoint, gripper).withTimeout(3).andThen(new MoveArmToFloor(firstJoint, secondJoint, compensation).withTimeout(2)).schedule();
                    } else {
                        new SwitchSides(firstJoint, secondJoint, gripper).withTimeout(3).andThen(new MoveArmToFloor(firstJoint, secondJoint, compensation).withTimeout(2)).schedule();

                    }
                })
        ));
        return eventMap;
    }
}
