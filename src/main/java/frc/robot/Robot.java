// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.spikes2212.dashboard.RootNamespace;
import com.spikes2212.util.Limelight;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    RootNamespace rootNamespace = new RootNamespace("root");
    Limelight limelight;
    NetworkTableInstance table;

    @Override
    public void robotInit() {
        limelight = new Limelight();
        table = NetworkTableInstance.getDefault();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        double[] result = table.getTable("limelight").getEntry("botpose").getDoubleArray(new double[]{});
        if(result.length>0) {
            Translation3d translation3d = new Translation3d(result[0], result[1], result[2]);
            Rotation3d rotation3d = new Rotation3d(result[3], result[4], result[5]);
            Pose3d pose3d = new Pose3d(translation3d, rotation3d);
            rootNamespace.putNumber("pose x", pose3d.getX());
            rootNamespace.putNumber("pose y", pose3d.getY());
        }
        result = table.getTable("limelight").getEntry("campose").getDoubleArray(new double[]{});
        if(result.length>0) {
            Translation3d translation3d = new Translation3d(result[0], result[1], result[2]);
            Rotation3d rotation3d = new Rotation3d(limelight.getHorizontalOffsetFromTargetInDegrees(),
                    limelight.getVerticalOffsetFromTargetInDegrees(), 0);
            Transform3d transform3d = new Transform3d(translation3d, rotation3d);
            rootNamespace.putNumber("transform x", transform3d.getX());
            rootNamespace.putNumber("transform y", transform3d.getY());
        }
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
