package frc.robot.subsystems;

import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.util.Limelight;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends DashboardedSubsystem {

    public static final String CAMERA_NAME = "photoncamera";

    public static final int PIPELINE_CONE_INDEX = 1;
    public static final int PIPELINE_CUBE_INDEX = 2;

    public static final int PIPELINE_RETROREFLECTIVE_INDEX = 1;
    public static final int PIPELINE_APRILTAG_INDEX = 2;

    private static Vision instance;

    private final PhotonCamera photonCamera;
    private final Limelight limelight;

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision(new PhotonCamera(CAMERA_NAME), new Limelight());
        }
        return instance;
    }

    private Vision(PhotonCamera photonCamera, Limelight limelight) {
        super("vision");
        this.photonCamera = photonCamera;
        this.limelight = limelight;
    }

    private double getPhotonVisionYaw() {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget().getYaw();
        }
        return 0; // if no target is detected
    }

    public void changePhotonVisionMode(boolean mode) {
        photonCamera.setDriverMode(mode);
    }

    public double getLimelightYaw() {
        return limelight.getHorizontalOffsetFromTargetInDegrees();
    }

    public boolean limelightHasTarget() {
        return limelight.isOnTarget();
    }

    public void changeCameraPipeline(int pipelineIndex) {
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
    }
}
