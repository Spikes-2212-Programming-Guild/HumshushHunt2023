package frc.robot.commands.athena;

public class Group4 extends ProgramBase {

    @Override
    public void writeProgram() {
          moveForwardInSeconds(3);
          moveArm();
          releaseCube();
        }
}
