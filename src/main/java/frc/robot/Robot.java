// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.spikes2212.command.drivetrains.commands.DriveArcade;
import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.UnifiedControlMode;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.KeepArmStable;
import frc.robot.commands.MoveArmToFloor;
import frc.robot.commands.PlaceGamePiece;
import frc.robot.commands.SwitchSides;
import frc.robot.services.ArmGravityCompensation;
import frc.robot.subsystems.ArmFirstJoint;
import frc.robot.subsystems.ArmSecondJoint;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;

import java.util.function.Supplier;

public class Robot extends TimedRobot {

    RootNamespace namespace = new RootNamespace("testing");
    Supplier<Double> speed = namespace.addConstantDouble("speed1", 0.2);
    Supplier<Double> setpoint = namespace.addConstantDouble("setpoint", 0);
    Supplier<Double> voltage = namespace.addConstantDouble("voltage", 0);
    Drivetrain drivetrain;
    ArmFirstJoint firstJoint = ArmFirstJoint.getInstance();
    ArmSecondJoint secondJoint = ArmSecondJoint.getInstance();
    Gripper gripper;
    private OI oi;
    RunCommand runCommand = new RunCommand(() -> secondJoint.setVoltage(voltage.get() * Math.cos(Math.toRadians(secondJoint.getAbsolutePosition()))));
    RunCommand runCommand1 = new RunCommand(() -> firstJoint.setVoltage(voltage.get()));
    private final Supplier<Double> firstSetpoint = namespace.addConstantDouble("first setpoint", 0);
    private final Supplier<Double> secondSetpoint = namespace.addConstantDouble("second setpoint", 0);
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
//        namespace.putData("move first joint in speed", new MoveFirstJoint(firstJoint, secondJoint,
//                ArmGravityCompensation.getInstance(), setpoint));
//        namespace.putData("move second joint in speed", new MoveSecondJoint(firstJoint, secondJoint,
//                ArmGravityCompensation.getInstance(), setpoint));
        namespace.putData("move position", new MoveSmartMotorControllerGenericSubsystem(secondJoint,
                secondJoint.getPIDSettings(), FeedForwardSettings.EMPTY_FFSETTINGS,
                UnifiedControlMode.POSITION, setpoint) {
            @Override
            public boolean isFinished() {
                return false;
            }
        });
        secondJoint.configureEncoders();
        namespace.putData("increase voltage", new MoveSmartMotorControllerGenericSubsystem(firstJoint,
                secondJoint.getPIDSettings(), secondJoint.getFeedForwardSettings(), UnifiedControlMode.VOLTAGE,
                setpoint) {
            @Override
            public boolean isFinished() {
                return false;
            }
        });
        namespace.putData("move both", new ParallelCommandGroup(
                new MoveSmartMotorControllerGenericSubsystem(secondJoint, secondJoint.getPIDSettings(),
                        secondJoint.getFeedForwardSettings(), UnifiedControlMode.POSITION, secondSetpoint) {
                    @Override
                    public boolean isFinished() {
                        return false;
                    }
                },
                new MoveSmartMotorControllerGenericSubsystem(firstJoint, firstJoint.getPIDSettings(),
                        firstJoint.getFeedForwardSettings(), UnifiedControlMode.POSITION, firstSetpoint) {
                    @Override
                    public boolean isFinished() {
                        return false;
                    }
                }
        ));
        namespace.putData("stop", new InstantCommand(() -> {
        }, firstJoint, secondJoint));
        namespace.putData("move arm", new PlaceGamePiece(firstJoint, secondJoint, PlaceGamePiece.ArmState.BACK_TOP));
        namespace.putData("switch sides BACK", new SwitchSides(firstJoint, secondJoint, gripper, true));
        namespace.putData("switch sides FRONT", new SwitchSides(firstJoint, secondJoint, gripper, false));
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
        new InstantCommand(() -> {}, firstJoint, secondJoint).ignoringDisable(true).schedule();
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
