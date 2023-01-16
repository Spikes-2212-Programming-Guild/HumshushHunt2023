package frc.robot.subsystems;

import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.util.Limelight;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends DashboardedSubsystem {

    public static final String CAMERA_NAME = "photoncamera";

    public static final int CAMERA_INDEX_CONE = 1;
    public static final int CAMERA_INDEX_CUBE = 2;

    public static Vision instance;

    private final PhotonCamera photonCamera;
    private final Limelight limelight;

    public static Vision getInstance() {
        if (instance==null) {
            instance = new Vision(new PhotonCamera(CAMERA_NAME), new Limelight());
        }
        return instance;
    }

    private Vision(PhotonCamera photonCamera, Limelight limelight) {
        super("camera");
        this.photonCamera = photonCamera;
        this.limelight = limelight;
    }

    private double getPhotonVisionYaw() {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget().getYaw();
        }
        return 10; // if no target is detected
    }

    public void changeMode(boolean mode) {
        photonCamera.setDriverMode(mode);
    }

    public double getLimelightYaw() {
        return limelight.getHorizontalOffsetFromTargetInDegrees();
    }

    public boolean limelightHasTarget() {
        return limelight.isOnTarget();
    }

    public void changeCameraPipeline() {
        if (photonCamera.getPipelineIndex()==CAMERA_INDEX_CONE) {
            photonCamera.setPipelineIndex(CAMERA_INDEX_CUBE);
        }

        else {
            photonCamera.setPipelineIndex(CAMERA_INDEX_CONE);
        }
    }

    @Override
    public void configureDashboard() {
        namespace.putBoolean("limelight has target", this::limelightHasTarget);
        namespace.putNumber("limelight yaw", this::getLimelightYaw);
        namespace.putNumber("photonvision yaw", this::getPhotonVisionYaw);
    }
}
