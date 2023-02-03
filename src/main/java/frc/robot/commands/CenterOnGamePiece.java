package frc.robot.commands;

import com.spikes2212.command.drivetrains.commands.DriveArcadeWithPID;
import frc.robot.subsystems.Drivetrain;
import frc.robot.services.VisionService;

public class CenterOnGamePiece extends DriveArcadeWithPID {

    private final VisionService vision;
    private final int pipeline;

    public CenterOnGamePiece(Drivetrain drivetrain, VisionService vision, int pipeline) {
        super(drivetrain, vision::getPhotonVisionYaw, 0, 0, drivetrain.getCameraPIDSettings());
        this.vision = vision;
        this.pipeline = pipeline;
    }

    @Override
    public void initialize() {
        vision.setPhotonVisionPipeline(pipeline);
    }
}
