package frc.robot.services;

import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.Limelight;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose3d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionService {

    public enum PhotonVisionPipeline {

        CONE(0), CUBE(1);

        public final int pipeline;

        PhotonVisionPipeline(int pipeline) {
            this.pipeline = pipeline;
        }
    }

    public enum LimelightPipeline {

        HIGH_RRT(0), LOW_RRT(1), APRIL_TAG(2);

        public final int pipeline;

        LimelightPipeline(int pipeline) {
            this.pipeline = pipeline;
        }
    }

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

    public void setPhotonVisionDriverMode(boolean mode) {
        photonCamera.setDriverMode(mode);
    }

    public boolean limelightHasTarget() {
        return limelight.hasTarget();
    }

    public void setPhotonVisionPipeline(PhotonVisionPipeline pipeline) {
        photonCamera.setPipelineIndex(pipeline.pipeline);
    }

    public void setLimelightPipeline(LimelightPipeline pipeline) {
        limelight.setPipeline(pipeline.pipeline);
    }

    public void configureDashboard() {
        namespace.putBoolean("limelight has target", this::limelightHasTarget);
        namespace.putNumber("limelight yaw", this::getLimelightYaw);
        namespace.putNumber("photon vision yaw", this::getPhotonVisionYaw);
        CameraServer.startAutomaticCapture();
    }
}
