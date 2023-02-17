package frc.robot.commands.autonomous;

import com.pathplanner.lib.auto.RamseteAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

import java.util.Map;

public class BasePathAuto extends RamseteAutoBuilder {

    public BasePathAuto(Drivetrain drivetrain, Map<String, Command> eventMap) {
        super(drivetrain::getPose2d, drivetrain::resetOdometry, drivetrain.getRamseteController(),
                drivetrain.getKinematics(), (leftMS, rightMS) -> drivetrain.setMetersPerSecond(leftMS, rightMS,
                        drivetrain.getLeftPIDSettings(), drivetrain.getRightPIDSettings(),
                        drivetrain.getFeedForwardSettings()), eventMap, true, drivetrain);
    }
}
