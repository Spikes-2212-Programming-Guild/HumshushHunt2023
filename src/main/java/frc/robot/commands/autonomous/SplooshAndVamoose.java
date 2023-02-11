package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.Climb;
import frc.robot.subsystems.Drivetrain;

import java.util.HashMap;
import java.util.Map;

public class SplooshAndVamoose extends BasePathAuto {

    private static final double MAX_VELOCITY = 1;
    private static final double MAX_ACCELERATION = 1;

    public SplooshAndVamoose(Drivetrain drivetrain) {
        super(drivetrain, getEventMap());
    }

    public CommandBase getCommand() {
        return fullAuto(PathPlanner.loadPath("Sploosh And Vamoose",
                new PathConstraints(MAX_VELOCITY, MAX_ACCELERATION)));
    }

    private static Map<String, Command> getEventMap() {
        Drivetrain drivetrain = Drivetrain.getInstance();
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("putGP", new SequentialCommandGroup(
            new PrintCommand("put gp"), new WaitCommand(1)
        ));
        eventMap.put("takeGP", new SequentialCommandGroup(
            new PrintCommand("take gp"), new WaitCommand(1)
        ));
        eventMap.put("climb", new Climb(drivetrain));
        return eventMap;
    }
}
