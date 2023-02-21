package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FakeArm extends SubsystemBase {

    private static FakeArm instance;

    public static FakeArm getInstance(){
        if(instance == null){
            instance = new FakeArm();
        }
        return instance;
    }
}
