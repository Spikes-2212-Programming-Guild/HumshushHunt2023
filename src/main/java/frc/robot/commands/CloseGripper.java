package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Gripper;

public class CloseGripper extends InstantCommand {

    private final Gripper gripper;

    public CloseGripper(Gripper gripper) {
        addRequirements(gripper);
        this.gripper = gripper;
    }

    @Override
    public void initialize() {
        gripper.closeGripper();
    }
}
