// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.spikes2212.command.drivetrains.commands.DriveArcade;
import com.spikes2212.command.drivetrains.commands.DriveTankWithPID;
import com.spikes2212.command.genericsubsystem.commands.smartmotorcontrollergenericsubsystem.MoveSmartMotorControllerGenericSubsystem;
import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.PlaystationControllerWrapper;
import com.spikes2212.util.UnifiedControlMode;
import com.spikes2212.util.XboxControllerWrapper;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.*;
import frc.robot.services.VisionService;
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
    PlaystationControllerWrapper ps = new PlaystationControllerWrapper(0);
    XboxControllerWrapper xbox = new XboxControllerWrapper(1);
    Drivetrain drivetrain;
    ArmFirstJoint firstJoint = ArmFirstJoint.getInstance();
    ArmSecondJoint secondJoint = ArmSecondJoint.getInstance();
    Gripper gripper;
    RunCommand runCommand = new RunCommand(() -> secondJoint.setVoltage(voltage.get() * Math.cos(Math.toRadians(secondJoint.getAbsolutePosition()))));
    RunCommand runCommand1 = new RunCommand(() -> firstJoint.setVoltage(voltage.get()));

    @Override
    public void robotInit() {
        Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
        namespace.putData("enable compressor", new InstantCommand(compressor::enableDigital));
        namespace.putData("disable compressor", new InstantCommand(compressor::disable));
        drivetrain = Drivetrain.getInstance();
//        firstJoint = ArmFirstJoint.getInstance();
//        secondJoint = ArmSecondJoint.getInstance();
        gripper = Gripper.getInstance();
//        DriveTankWithPID drive = new DriveTankWithPID(drivetrain, drivetrain.getLeftPIDSettings(), drivetrain.getRightPIDSettings(),
//                setpoint, setpoint, drivetrain::getLeftPosition, drivetrain::getRightPosition);
//        xbox.getYellowButton().onTrue(drive);
        xbox.getBlueButton().onTrue(new InstantCommand(drivetrain::resetEncoders));
//        ps.getCircleButton().whileTrue(new SetSparkMax(new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless), speed));
//        ps.getCrossButton().whileTrue(new SetSparkMax(new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless), speed));
//        ps.getSquareButton().whileTrue(new SetSparkMax(new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless), speed));
//        ps.getTriangleButton().whileTrue(new SetSparkMax(new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless), speed));
        ps.getR1Button().whileTrue(new MoveSmartMotorControllerGenericSubsystem(firstJoint, firstJoint.getPIDSettings(), firstJoint.getFeedForwardSettings(), UnifiedControlMode.PERCENT_OUTPUT, firstJoint.forwardSpeed) {
            @Override
            public boolean isFinished() {
                return false;
            }
        });
        ps.getR2Button().whileTrue(new MoveSmartMotorControllerGenericSubsystem(firstJoint, firstJoint.getPIDSettings(), firstJoint.getFeedForwardSettings(), UnifiedControlMode.PERCENT_OUTPUT, firstJoint.backwardsSpeed) {
            @Override
            public boolean isFinished() {
                return false;
            }
        });
        ps.getL2Button().whileTrue(new MoveSmartMotorControllerGenericSubsystem(secondJoint, secondJoint.getPIDSettings(), secondJoint.getFeedForwardSettings(), UnifiedControlMode.PERCENT_OUTPUT, secondJoint.backwardsSpeed) {
            @Override
            public boolean isFinished() {
                return false;
            }
        });
        ps.getL1Button().whileTrue(new MoveSmartMotorControllerGenericSubsystem(secondJoint, secondJoint.getPIDSettings(), secondJoint.getFeedForwardSettings(), UnifiedControlMode.PERCENT_OUTPUT, secondJoint.forwardSpeed) {
            @Override
            public boolean isFinished() {
                return false;
            }
        });
//        CANSparkMax sparkMax = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
//        sparkMax.setInverted(true);x
//        ps.getR2Button().whileTrue(new SetSparkMax(new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless), speed));
//        ps.getR1Button().whileTrue(new SetSparkMax(sparkMax, speed));
//        ps.getL1Button().whileTrue(new SetSparkMax(new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless), -speed.get()));
        ps.getUpButton().onTrue(new OpenGripper(gripper));
        ps.getDownButton().onTrue(new CloseGripper(gripper));
        ps.getLeftButton().onTrue(new InstantCommand(() -> drivetrain.setMode(CANSparkMax.IdleMode.kCoast)));
        ps.getTriangleButton().onTrue(new Climb(drivetrain));
        xbox.getRightButton().onTrue(new InstantCommand(() -> {
        }, drivetrain));
        xbox.getButtonStart().onTrue(new InstantCommand(() -> drivetrain.setMode(CANSparkMax.IdleMode.kCoast)));
        xbox.getButtonBack().onTrue(new InstantCommand(() -> drivetrain.setMode(CANSparkMax.IdleMode.kBrake)));
        xbox.getLeftButton().onTrue(new CenterOnRRT(drivetrain, VisionService.getInstance(), VisionService.LimelightPipeline.HIGH_RRT));
        namespace.putRunnable("coast arm", () -> {
            firstJoint.setIdleMode(CANSparkMax.IdleMode.kCoast);
            secondJoint.setIdleMode(CANSparkMax.IdleMode.kCoast);
        });
        xbox.getRightButton().whileTrue(runCommand);
        xbox.getUpButton().whileTrue(runCommand1);
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
        runCommand.cancel();
        runCommand1.cancel();
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

    private double getRightY() {
        double val = xbox.getRightY();
        return Math.signum(val) * val * val;
    }

    private double getLeftX() {
        double val = xbox.getLeftX();
        return Math.signum(val) * val * val;
    }

    @Override
    public void teleopInit() {
        drivetrain.setDefaultCommand(new DriveArcade(drivetrain, this::getRightY, this::getLeftX));
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
