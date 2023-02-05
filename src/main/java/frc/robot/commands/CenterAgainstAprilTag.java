package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.spikes2212.util.Limelight;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Drivetrain;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Optional;

public class CenterAgainstAprilTag extends CommandBase {

    private Pose3d prevEstimatedRobotPose;

    PPRamseteCommand command;

    //it's a trash code but ignore it
    public CenterAgainstAprilTag(Drivetrain drivetrain) throws IOException {
//        Limelight limelight = new Limelight();
        PhotonCamera photonCamera = new PhotonCamera("photonvision");
        int id = photonCamera.getLatestResult().getBestTarget().getFiducialId();
        AprilTagFieldLayout layout = new AprilTagFieldLayout(Path.of(Filesystem.getDeployDirectory().getPath(), "field.json")); //need to check if it works asa well as upload it to the deploy directory
        Optional<Pose3d> optionalPose = layout.getTagPose(id);
        if (optionalPose.isPresent()) {
            Pose3d tagPose = layout.getTagPose(id).get();
//            Pose3d currentPose = limelight.getRobotPose();
            PhotonPoseEstimator estimator = new PhotonPoseEstimator(layout, PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY,
                    photonCamera, new Transform3d(new Translation3d(0.34, 0, 0.525), new Rotation3d()));
            PathPoint endPoint;
            if (id <= 4) {
                endPoint = new PathPoint(new Translation2d(tagPose.getX() - 1, tagPose.getY()), tagPose.getRotation().toRotation2d());
            } else {
                endPoint = new PathPoint(new Translation2d(tagPose.getX() + 1, tagPose.getY()), tagPose.getRotation().toRotation2d());
            }
            Optional<EstimatedRobotPose> currentPose = estimator.update();
            assert currentPose.isPresent();
            PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                    new PathConstraints(1, 1),
                    new PathPoint(new Translation2d(currentPose.get().estimatedPose.getX(), currentPose.get().estimatedPose.getY()),
                            currentPose.get().estimatedPose.getRotation().toRotation2d()),
                    endPoint
            );
            command = new PPRamseteCommand(trajectory, drivetrain::getPose2d, drivetrain.getRamseteController(), drivetrain.getKinematics(),
                    (leftMS, rightMS) -> drivetrain.setMetersPerSecond(leftMS, rightMS, drivetrain.getLeftPIDSettings(),
                            drivetrain.getRightPIDSettings(), drivetrain.getFeedForwardSettings()), drivetrain);
            addRequirements(command.getRequirements().toArray(new Subsystem[0]));
        }
    }


    @Override
    public void initialize() {
        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }
}
