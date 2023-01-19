package frc.robot.subsystems;

import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.util.Limelight;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends DashboardedSubsystem {

    public static final String PHOTON_VISION_CAMERA_NAME = "photoncamera";

    public static final int PIPELINE_CONE_INDEX = 0;
    public static final int PIPELINE_CUBE_INDEX = 1;

    public static final int PIPELINE_RETROREFLECTIVE_INDEX = 0;
    public static final int PIPELINE_APRILTAG_INDEX = 1;

    private static Vision instance;

    private final Limelight limelight;
    private final PhotonCamera photonCamera;

    public static Vision getInstance(){
        if (instance==null){
            instance = new Vision(new PhotonCamera(PHOTON_VISION_CAMERA_NAME), new Limelight());
        }
        return instance;
    }

    private Vision(PhotonCamera photonCamera, Limelight limelight) {
        super("vision");
        this.limelight = limelight;
        this.photonCamera = photonCamera;
    }

    public void setDriverMode(boolean mode) {
        photonCamera.setDriverMode(mode);
    }

    private double getPhotonvisionYaw(){
        PhotonPipelineResult result = photonCamera.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget().getYaw();
        }
        return 0;
    }

    private double getLimelightYaw(){
        return limelight.getHorizontalOffsetFromTargetInDegrees();
    }

    public boolean limelightHasTarget(){
        return limelight.hasTarget();
    }

    public void changePhotonVisionPipeline(int pipelineIndex){
        photonCamera.setPipelineIndex(pipelineIndex);
    }

    public void changeLimelightPipeline(int pipelineIndex){
        limelight.setPipeline(pipelineIndex);
    }

    @Override
    public void configureDashboard() {
        namespace.putBoolean("limelight has target", this::limelightHasTarget);
        namespace.putNumber("photonvision yaw", this::getPhotonvisionYaw);
        namespace.putNumber("limelight yaw", this::getLimelightYaw);
    }
}
