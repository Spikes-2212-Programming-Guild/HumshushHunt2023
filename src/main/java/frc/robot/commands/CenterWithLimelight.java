package frc.robot.commands;

import com.spikes2212.command.drivetrains.commands.DriveArcadeWithPID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.services.VisionService;
import frc.robot.services.VisionService.LimelightPipeline;
import frc.robot.subsystems.Drivetrain;

public class CenterWithLimelight extends DriveArcadeWithPID {

    private final VisionService vision;
    private final LimelightPipeline pipeline;

    public CenterWithLimelight(Drivetrain drivetrain, VisionService vision, LimelightPipeline pipeline) {
        super(drivetrain, vision::getLimelightYaw, 0, 0, drivetrain.getCameraPIDSettings(),
                drivetrain.getFeedForwardSettings());
        this.vision = vision;
        this.pipeline = pipeline;
    }

    @Override
    public void initialize() {
        feedForwardSettings.setkG(() -> (3.1 / RobotController.getBatteryVoltage()) * -Math.signum(vision.getLimelightYaw()));
        vision.setLimelightPipeline(pipeline);
    }

    @Override
    public void end(boolean interrupted) {
        feedForwardSettings.setkG(() -> 0.0);
        super.end(interrupted);
    }
}
