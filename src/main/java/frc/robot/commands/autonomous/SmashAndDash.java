package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Drivetrain;

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
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("putGP", new SequentialCommandGroup(
                new PrintCommand("putGP"),
                new WaitCommand(1)
        ));
        eventMap.put("takeGP", new SequentialCommandGroup(
                new PrintCommand("takeGP"),
                new WaitCommand(1)
        ));
        eventMap.put("putGP2", new SequentialCommandGroup(
                new PrintCommand("done")
        ));
        return eventMap;
    }
}
