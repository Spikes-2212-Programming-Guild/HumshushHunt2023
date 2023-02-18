// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.spikes2212.command.drivetrains.commands.DriveArcade;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.KeepArmStable;
import frc.robot.commands.MoveArmToFloor;
import frc.robot.commands.PlaceGamePiece;
import frc.robot.commands.SwitchSides;
import frc.robot.services.ArmGravityCompensation;
import frc.robot.subsystems.ArmFirstJoint;
import frc.robot.subsystems.ArmSecondJoint;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;

public class Robot extends TimedRobot {

    private final RootNamespace namespace = new RootNamespace("robot");
    private Drivetrain drivetrain;
    private ArmFirstJoint firstJoint = ArmFirstJoint.getInstance();
    private ArmSecondJoint secondJoint = ArmSecondJoint.getInstance();
    private Gripper gripper;
    private OI oi;
    private ArmGravityCompensation compensation;

    @Override
    public void robotInit() {
        Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
        namespace.putData("enable compressor", new InstantCommand(compressor::enableDigital));
        namespace.putData("disable compressor", new InstantCommand(compressor::disable));
        oi = OI.getInstance();
        drivetrain = Drivetrain.getInstance();
        compensation = ArmGravityCompensation.getInstance();
        firstJoint = ArmFirstJoint.getInstance();
        secondJoint = ArmSecondJoint.getInstance();
        gripper = Gripper.getInstance();
        namespace.putRunnable("coast arm", () -> {
            firstJoint.setIdleMode(CANSparkMax.IdleMode.kCoast);
            secondJoint.setIdleMode(CANSparkMax.IdleMode.kCoast);
        });
        namespace.putData("keep arm stable", new KeepArmStable(firstJoint, secondJoint, ArmGravityCompensation.getInstance()));
        namespace.putData("stop", new InstantCommand(() -> {
        }, firstJoint, secondJoint));
        namespace.putData("move arm", new PlaceGamePiece(firstJoint, secondJoint, PlaceGamePiece.ArmState.BACK_TOP));
        namespace.putData("switch arm sides", new SwitchSides(firstJoint, secondJoint, gripper));
        namespace.putData("floor back", new MoveArmToFloor(firstJoint, secondJoint, compensation, PlaceGamePiece.ArmState.FLOOR_BACK));
        namespace.putData("floor front", new MoveArmToFloor(firstJoint, secondJoint, compensation, PlaceGamePiece.ArmState.FLOOR_FRONT));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        namespace.update();
        drivetrain.periodic();
        firstJoint.periodic();
        secondJoint.periodic();
        gripper.periodic();
    }

    @Override
    public void disabledInit() {
        new InstantCommand(() -> {
        }, firstJoint, secondJoint).ignoringDisable(true).schedule();
    }

  @Override
  public void disabledPeriodic() {

    }

    @Override
    public void autonomousInit() {
        new InstantCommand(() -> {
        }, firstJoint, secondJoint).schedule();
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        new InstantCommand(() -> {
        }, firstJoint, secondJoint).schedule();
        drivetrain.setDefaultCommand(new DriveArcade(drivetrain, oi::getRightY, oi::getLeftX));
    }

    @Override
    public void teleopPeriodic() {
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
