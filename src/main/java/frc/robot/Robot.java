// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.spikes2212.command.drivetrains.commands.DriveArcade;
import com.spikes2212.util.UnifiedControlMode;
import com.spikes2212.util.XboxControllerWrapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

import java.util.HashMap;

public class Robot extends TimedRobot {

    XboxControllerWrapper xbox = new XboxControllerWrapper(1);

    @Override
    public void robotInit() {
        Drivetrain drivetrain = Drivetrain.getInstance();
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("move", new PathConstraints(1, 1));
        RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(drivetrain::getPose2d, drivetrain::resetOdometry,
                drivetrain.getRamseteController(), drivetrain.getKinematics(),
                (leftSpeed, rightSpeed) -> drivetrain.pidSet(UnifiedControlMode.VELOCITY, leftSpeed, rightSpeed, drivetrain.getLeftPIDSettings(),
                        drivetrain.getRightPIDSettings(), drivetrain.getFeedForwardSettings()), new HashMap<>(),
                false, drivetrain);
        CommandBase command = autoBuilder.followPath(trajectory);
//        PathPlannerTrajectory trajectory = PathPlanner.loadPath("move", new PathConstraints(1, 0.5));
//        PPRamseteCommand command = new PPRamseteCommand(trajectory, drivetrain::getPose2d,
//                drivetrain.getRamseteController(), drivetrain.getKinematics(), (leftMS, rightMS) -> drivetrain.setMetersPerSecond(leftMS, rightMS,
//                drivetrain.getLeftPIDSettings(), drivetrain.getRightPIDSettings(),
//                drivetrain.getFeedForwardSettings()), false, drivetrain);
        xbox.getYellowButton().onTrue(command);
        xbox.getBlueButton().onTrue(new InstantCommand(drivetrain::resetEncoders));
        xbox.getGreenButton().onTrue(new InstantCommand(() -> drivetrain.resetOdometry(new Pose2d(0, 0, new Rotation2d()))));
        xbox.getRedButton().onTrue(new InstantCommand(drivetrain::resetGyro));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
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
