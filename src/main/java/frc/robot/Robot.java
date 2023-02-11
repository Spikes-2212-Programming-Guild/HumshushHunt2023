// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.command.drivetrains.commands.DriveArcade;
import com.spikes2212.command.drivetrains.commands.smartmotorcontrollerdrivetrain.MoveSmartMotorControllerTankDrivetrain;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import com.spikes2212.util.XboxControllerWrapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.autonomous.SmashAndDash;
import frc.robot.subsystems.Drivetrain;

public class Robot extends TimedRobot {

    private final RootNamespace root = new RootNamespace("root");

    XboxControllerWrapper xbox = new XboxControllerWrapper(1);

    @Override
    public void robotInit() {
        new Compressor(PneumaticsModuleType.CTREPCM).enableDigital();
        Drivetrain drivetrain = Drivetrain.getInstance();
        CommandBase command = new SmashAndDash(drivetrain).getCommand();
        CommandBase moveStraight = new MoveSmartMotorControllerTankDrivetrain(drivetrain, drivetrain.getLeftPIDSettings(),
                drivetrain.getRightPIDSettings(), drivetrain.getFeedForwardSettings(), UnifiedControlMode.VELOCITY,
                () -> 2.0, () -> 2.0) {
            @Override
            public boolean isFinished() {
                return false;
            }
        };
        root.putData("move straight", moveStraight);

//        PathPlannerTrajectory trajectory = PathPlanner.loadPath("move", new PathConstraints(1, 0.5));
//        PPRamseteCommand command = new PPRamseteCommand(trajectory, drivetrain::getPose2d,
//                drivetrain.getRamseteController(), drivetrain.getKinematics(), (leftMS, rightMS) -> drivetrain.setMetersPerSecond(leftMS, rightMS,
//                drivetrain.getLeftPIDSettings(), drivetrain.getRightPIDSettings(),
//                drivetrain.getFeedForwardSettings()), false, drivetrain);
        xbox.getYellowButton().onTrue(command);
        xbox.getBlueButton().onTrue(new InstantCommand(drivetrain::resetEncoders));
        xbox.getGreenButton().onTrue(new InstantCommand(() -> drivetrain.resetOdometry(new Pose2d(0, 0, new Rotation2d()))));
        xbox.getRedButton().onTrue(new InstantCommand(drivetrain::resetGyro));
        xbox.getButtonStart().onTrue(moveStraight);
        xbox.getRTButton().onTrue(new InstantCommand(() -> {}, drivetrain));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        root.update();
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        Drivetrain drivetrain = Drivetrain.getInstance();
        drivetrain.setDefaultCommand(new DriveArcade(drivetrain, this::getRightY, this::getLeftX));
    }

    @Override
    public void teleopPeriodic() {
    }

    private double getLeftX() {
        double val = xbox.getLeftX();
        return val * val * Math.signum(val);
    }

    private double getRightY() {
        double val = xbox.getRightY();
        return val * val * Math.signum(val);
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void simulationInit() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
