package frc.robot.subsystems;

import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.util.Limelight;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose3d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends DashboardedSubsystem {
//
//    private final CameraServer cameraServer;
//    private final UsbCamera usbCamera;
//    private final CvSink cvSink;
//    private final CvSource cvSource;

    public static final String PHOTON_VISION_CAMERA_NAME = "photoncamera";

    public static final int PIPELINE_CONE_INDEX = 0;
    public static final int PIPELINE_CUBE_INDEX = 1;

    public static final int PIPELINE_HIGH_RRT_INDEX = 0;
    public static final int PIPELINE_LOW_RRT_INDEX = 1;
    public static final int PIPELINE_APRILTAG_INDEX = 2;

    private static Vision instance;

    private final PhotonCamera photonCamera;
    private final Limelight limelight;

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision("vision", new PhotonCamera(PHOTON_VISION_CAMERA_NAME), new Limelight());
        }
        return instance;
    }

    private Vision(String namespaceName, PhotonCamera photonCamera, Limelight limelight) {
        super(namespaceName);
        this.photonCamera = photonCamera;
        this.limelight = limelight;
        configureDashboard();
    }

    public double getPhotonVisionYaw() {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget().getYaw();
        }
        return 0; // if no target is detected
    }

    public Pose3d getRobotPose() {
        return limelight.getRobotPose();
    }

    public long getAprilTagID() {
        return limelight.getID();
    }

    public void changePhotonVisionMode(boolean mode) {
        photonCamera.setDriverMode(mode);
    }

    public double getLimelightYaw() {
        return limelight.getHorizontalOffsetFromTargetInDegrees();
    }

    public boolean limelightHasTarget() {
        return limelight.hasTarget();
    }

    public void changePhotonVisionPipeline(int pipelineIndex) {
        photonCamera.setPipelineIndex(pipelineIndex);
    }

    public void changeLimelightPipeline(int pipelineIndex) {
        limelight.setPipeline(pipelineIndex);
    }

    @Override
    public void configureDashboard() {
        namespace.putBoolean("limelight has target", this::limelightHasTarget);
        namespace.putNumber("limelight yaw", this::getLimelightYaw);
        namespace.putNumber("photonvision yaw", this::getPhotonVisionYaw);
        CameraServer.startAutomaticCapture();
    }
}
