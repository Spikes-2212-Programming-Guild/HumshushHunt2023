package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.spikes2212.util.Limelight;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

import java.io.IOException;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

public class CenterAgainstAprilTag extends SequentialCommandGroup {

    Limelight limelight;

    //it's a trash code but ignore it
    public CenterAgainstAprilTag(Drivetrain drivetrain, PathPlannerTrajectory trajectory, Supplier<Pose2d> poseSupplier,
                                 RamseteController controller, SimpleMotorFeedforward feedforward,
                                 DifferentialDriveKinematics kinematics,
                                 Supplier<DifferentialDriveWheelSpeeds> speedsSupplier, PIDController leftController,
                                 PIDController rightController, BiConsumer<Double, Double> outputVolts) throws IOException {
        limelight = new Limelight();
        AprilTagFieldLayout layout = new AprilTagFieldLayout("path");
        Pose3d tagPose = layout.getTagPose((int) limelight.getID()).get();
        Pose3d currentPose = limelight.getRobotPose();
        trajectory = PathPlanner.generatePath(
                new PathConstraints(1, 1),
                new PathPoint(new Translation2d(currentPose.getX(), currentPose.getY()), currentPose.getRotation().toRotation2d()),
                new PathPoint(new Translation2d(tagPose.getX(), tagPose.getY()), tagPose.getRotation().toRotation2d())
        );
        //used sequential because i wanted to do things before calling super so i didn't inherit PPRamseteCommand
        //and i wanted to do it all in one class for comfortability
        addCommands(new PPRamseteCommand(trajectory, drivetrain::getPose2d, drivetrain.getRamseteController(), drivetrain.getKinematics(),
                (leftMS, rightMS) -> drivetrain.setMetersPerSecond(leftMS, rightMS, drivetrain.getLeftPIDSettings(),
                        drivetrain.getRightPIDSettings(), drivetrain.getFeedForwardSettings()), drivetrain));

    }
}
