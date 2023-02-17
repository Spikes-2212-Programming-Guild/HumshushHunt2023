package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.CenterWithLimelight;
import frc.robot.commands.CloseGripper;
import frc.robot.commands.PlaceGamePiece;
import frc.robot.services.VisionService;
import frc.robot.subsystems.ArmFirstJoint;
import frc.robot.subsystems.ArmSecondJoint;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;

import java.util.HashMap;
import java.util.Map;

public class SmashAndDash extends BasePathAuto {

    private static final double MAX_VELOCITY = 2.7;
    private static final double MAX_ACCELERATION = 3;

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
        Gripper gripper = Gripper.getInstance();
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("putGP", new SequentialCommandGroup(
                new CenterWithLimelight(drivetrain, vision, VisionService.LimelightPipeline.HIGH_RRT).withTimeout(1),
                new PlaceGamePiece(firstJoint, secondJoint, PlaceGamePiece.ArmState.BACK_TOP)
        ));
        eventMap.put("takeGP", new SequentialCommandGroup(
                new WaitCommand(2),
                new CloseGripper(gripper)
        ));
        eventMap.put("putGP2", new SequentialCommandGroup(
                new CenterWithLimelight(drivetrain, vision, VisionService.LimelightPipeline.APRIL_TAG).withTimeout(1),
                new PlaceGamePiece(firstJoint, secondJoint, PlaceGamePiece.ArmState.BACK_TOP)
        ));
        return eventMap;
    }
}
