package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.spikes2212.command.drivetrains.commands.DriveArcadeWithPID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.services.VisionService;
import frc.robot.subsystems.ArmFirstJoint;
import frc.robot.subsystems.ArmSecondJoint;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;

import java.util.HashMap;
import java.util.Map;

public class PlanB extends BasePathAuto {

    private static final double MAX_VELOCITY = 1.5;
    private static final double MAX_ACCELERATION = 2;

    public PlanB(Drivetrain drivetrain) {
        super(drivetrain, getEventMap());
    }

    public CommandBase getCommand() {
        return fullAuto(PathPlanner.loadPathGroup("Plan B", true,
                new PathConstraints(MAX_VELOCITY, MAX_ACCELERATION)));
    }

    private static Map<String, Command> getEventMap() {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("putGP", new SequentialCommandGroup(
                new PrintCommand("put gp"),
                new PlaceGamePiece(ArmFirstJoint.getInstance(), ArmSecondJoint.getInstance(),
                        PlaceGamePiece.ArmState.FRONT_TOP),
                new OpenGripper(Gripper.getInstance()),
                new MoveSecondJoint(ArmSecondJoint.getInstance(), () -> PlaceGamePiece.ArmState.FOLD_ABOVE_180.secondJointPosition, () -> 0.005,
                        () -> PlaceGamePiece.ArmState.FOLD_ABOVE_180.moveDuration + 0.2),
                new CloseGripper(Gripper.getInstance()),
                new MoveFirstJoint(ArmFirstJoint.getInstance(), () -> 110.0, () -> 0.005,
                        () -> PlaceGamePiece.ArmState.FOLD_ABOVE_180.moveDuration + 0.2)
        ));
        eventMap.put("takeGP", new SequentialCommandGroup(
                new PrintCommand("take gp"), new InstantCommand()
        ));
        return eventMap;
    }
}
