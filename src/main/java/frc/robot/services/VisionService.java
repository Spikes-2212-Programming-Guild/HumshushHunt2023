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
    private static final String FRONT_LIMELIGHT_NAME = "limelight-front";
    private static final String BACK_LIMELIGHT_NAME = "limelight";

    private static final double TOLERANCE = 1.5;

    private static VisionService instance;

    private final RootNamespace namespace;

    private final PhotonCamera photonCamera;
    private final Limelight frontLimelight;
    private final Limelight backLimelight;

    public static VisionService getInstance() {
        if (instance == null) {
            instance = new VisionService("vision", new PhotonCamera(PHOTON_VISION_CAMERA_NAME), new Limelight(FRONT_LIMELIGHT_NAME), new Limelight(BACK_LIMELIGHT_NAME));
        }
        return instance;
    }

    private VisionService(String namespaceName, PhotonCamera photonCamera, Limelight frontLimelight, Limelight backLimelight) {
        this.namespace = new RootNamespace(namespaceName);
        this.photonCamera = photonCamera;
        this.frontLimelight = frontLimelight;
        this.backLimelight = backLimelight;
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
        return frontLimelight.getRobotPose();
    }

    public long getAprilTagID() {
        return frontLimelight.getID();
    }

    public double getFrontLimelightYaw() {
        return frontLimelight.getHorizontalOffsetFromTargetInDegrees();
    }

    public double getBackLimelightYaw() {
        return backLimelight.getHorizontalOffsetFromTargetInDegrees();
    }

    public void setPhotonVisionDriverMode(boolean mode) {
        photonCamera.setDriverMode(mode);
    }

    public boolean frontLimelightHasTarget() {
        return frontLimelight.hasTarget();
    }

    public boolean backLLimelightHasTarget() {
        return backLimelight.hasTarget();
    }

    public boolean frontLimelightCentered() {
        if (frontLimelight.hasTarget()) {
            return (Math.abs(frontLimelight.getHorizontalOffsetFromTargetInDegrees()) <= TOLERANCE);
        }
        return false;
    }

    public boolean backLimelightCentered() {
        if (backLimelight.hasTarget()) {
            return (Math.abs(backLimelight.getHorizontalOffsetFromTargetInDegrees()) <= TOLERANCE);
        }
        return false;
    }

    public boolean photonVisionCentered() {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        if (result.hasTargets()) {
            return (Math.abs(result.getBestTarget().getYaw()) <= TOLERANCE);
        }
        return false;
    }

    public void setPhotonVisionPipeline(PhotonVisionPipeline pipeline) {
        photonCamera.setPipelineIndex(pipeline.pipeline);
    }

    public void setFrontLimelightPipeline(LimelightPipeline pipeline) {
        frontLimelight.setPipeline(pipeline.pipeline);
    }

    public void setBackLimelightPipeline(LimelightPipeline pipeline) {
        backLimelight.setPipeline(pipeline.pipeline);
    }

    public void configureDashboard() {
        namespace.putBoolean("front limelight has target", this::frontLimelightHasTarget);
        namespace.putBoolean("back limelight has target", this::backLLimelightHasTarget);
        namespace.putNumber("front limelight yaw", this::getFrontLimelightYaw);
        namespace.putNumber("back limelight yaw", this::getBackLimelightYaw);
        namespace.putNumber("photon vision yaw", this::getPhotonVisionYaw);
        namespace.putBoolean("front limelight centered", this::frontLimelightCentered);
        namespace.putBoolean("back limelight centered", this::backLimelightCentered);
//        CameraServer.startAutomaticCapture();
    }
}
