package frc.robot.commands;

import com.spikes2212.command.drivetrains.commands.DriveArcadeWithPID;
import frc.robot.services.VisionService;
import frc.robot.services.VisionService.PhotonVisionPipeline;
import frc.robot.subsystems.Drivetrain;

public class CenterOnGamePiece extends DriveArcadeWithPID {

    private final VisionService vision;
    private final PhotonVisionPipeline pipeline;

    public CenterOnGamePiece(Drivetrain drivetrain, VisionService vision, PhotonVisionPipeline pipeline) {
        super(drivetrain, vision::getPhotonVisionYaw, 0, 0, drivetrain.getCameraPIDSettings());
        this.vision = vision;
        this.pipeline = pipeline;
    }

    @Override
    public void initialize() {
        vision.setPhotonVisionPipeline(pipeline);
    }
}
