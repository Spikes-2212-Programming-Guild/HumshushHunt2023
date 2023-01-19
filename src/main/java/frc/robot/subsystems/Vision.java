package frc.robot.subsystems;

import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.util.Limelight;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends DashboardedSubsystem {

    private static Vision instance;

    public static final String CAMERA_NAME = "photoncamera";

    public static final int PIPELINE_CONE_INDEX = 1;
    public static final int PIPELINE_CUBE_INDEX = 2;

    public static final int PIPELINE_RETROREFLECTIVE_INDEX = 1;
    public static final int PIPELINE_APRILTAG_INDEX = 2;

    private final Limelight limelight;
    private final PhotonCamera photonCamera;

    public static Vision getInstance(){
        if (instance==null){
            instance = new Vision(new PhotonCamera(CAMERA_NAME), new Limelight());
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

    public boolean limelightIsOnTarget(){
        return limelight.isOnTarget();
    }

    public boolean photonvisionIsOnTarget(){
        PhotonPipelineResult result = photonCamera.getLatestResult();
        return result.hasTargets();
    }

    public void changePhotonVisionPipeline(int pipelineIndex){
        photonCamera.setPipelineIndex(pipelineIndex);
    }

    public void changeLimelightPipeline(int pipelineIndex){
        limelight.setPipeline(pipelineIndex);
    }

    @Override
    public void configureDashboard() {
        namespace.putBoolean("limelight has target", this::limelightIsOnTarget);
        namespace.putBoolean("photonvision has target", this::photonvisionIsOnTarget);
        namespace.putNumber("photonvision yaw", this::getPhotonvisionYaw);
        namespace.putNumber("limelight yaw", this::getLimelightYaw);
    }
}
