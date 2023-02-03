package frc.robot.services;

import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.Limelight;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose3d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionService {

    public static final int PIPELINE_CONE_INDEX = 0;
    public static final int PIPELINE_CUBE_INDEX = 1;

    public static final int PIPELINE_HIGH_RRT_INDEX = 0;
    public static final int PIPELINE_LOW_RRT_INDEX = 1;
    public static final int PIPELINE_APRILTAG_INDEX = 2;

    private static final String PHOTON_VISION_CAMERA_NAME = "photonvision";

    private static VisionService instance;

    private final RootNamespace namespace;

    private final PhotonCamera photonCamera;
    private final Limelight limelight;

    public static VisionService getInstance() {
        if (instance == null) {
            instance = new VisionService("vision", new PhotonCamera(PHOTON_VISION_CAMERA_NAME), new Limelight());
        }
        return instance;
    }

    private VisionService(String namespaceName, PhotonCamera photonCamera, Limelight limelight) {
        this.namespace = new RootNamespace(namespaceName);
        this.photonCamera = photonCamera;
        this.limelight = limelight;
        configureDashboard();
    }

    public void periodic() {
        namespace.update();
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

    public double getLimelightYaw() {
        return limelight.getHorizontalOffsetFromTargetInDegrees();
    }

    public void changePhotonVisionDriverMode(boolean mode) {
        photonCamera.setDriverMode(mode);
    }

    public boolean limelightHasTarget() {
        return limelight.hasTarget();
    }

    public void setPhotonVisionPipeline(int pipelineIndex) {
        photonCamera.setPipelineIndex(pipelineIndex);
    }

    public void setLimelightPipeline(int pipelineIndex) {
        limelight.setPipeline(pipelineIndex);
    }

    public void configureDashboard() {
        namespace.putBoolean("limelight has target", this::limelightHasTarget);
        namespace.putNumber("limelight yaw", this::getLimelightYaw);
        namespace.putNumber("photonvision yaw", this::getPhotonVisionYaw);
        CameraServer.startAutomaticCapture();
    }
}
