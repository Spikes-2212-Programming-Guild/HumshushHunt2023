package frc.robot.commands;

import com.spikes2212.command.drivetrains.commands.DriveArcadeWithPID;
import edu.wpi.first.wpilibj.RobotController;
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
        feedForwardSettings.setkG(() -> (3.1 / RobotController.getBatteryVoltage()) * -Math.signum(vision.getPhotonVisionYaw()));
    }

    @Override
    public void end(boolean interrupted) {
        feedForwardSettings.setkG(() -> 0.0);
        super.end(interrupted);
    }
}
