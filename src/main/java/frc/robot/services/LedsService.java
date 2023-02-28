package frc.robot.services;

import com.ctre.phoenix.CANifier;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotMap;
import frc.robot.subsystems.ArmSecondJoint;
import frc.robot.subsystems.Gripper;

public class LedsService {

    public enum Mode {

        OFF(0, 0, 0), EMPTY_GRIPPER(254, 0, 0), ALLIGNED_TO_GAME_PIECE(204, 0, 254), HAS_GAME_PIECE(0, 0, 254),
        HAS_GAME_PIECE_AND_ALLIGNED(0, 254, 0);

        public final int red;
        public final int green;
        public final int blue;

        Mode(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }
    }

    private static final int NUMBER_OF_LEDS = 60;

    private static LedsService instance;

    private final RootNamespace namespace;

    private final ArmSecondJoint secondJoint;
    private final Gripper gripper;

    private final VisionService vision;

    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    public static LedsService getInstance() {
        if (instance == null) {
            instance = new LedsService("leds", new AddressableLED(RobotMap.PWM.LED_PORT),
                    new AddressableLEDBuffer(NUMBER_OF_LEDS), ArmSecondJoint.getInstance(), VisionService.getInstance(), Gripper.getInstance());
        }
        return instance;
    }

    private LedsService(String namespaceName, AddressableLED led, AddressableLEDBuffer ledBuffer, ArmSecondJoint secondJoint, VisionService vision, Gripper gripper) {
        namespace = new RootNamespace(namespaceName);
        this.led = led;
        this.ledBuffer = ledBuffer;
        this.secondJoint = secondJoint;
        this.vision = vision;
        this.gripper = gripper;
        led.setLength(ledBuffer.getLength());
        led.start();
        configureDashboard();
        CANifier caNifier = new CANifier(8);
//        caNifier.setLEDOutput(1, CANifier.LEDChannel.LEDChannelA);
//        caNifier.setLEDOutput(1, CANifier.LEDChannel.LEDChannelB);
//        caNifier.setLEDOutput(1, CANifier.LEDChannel.LEDChannelC);
    }

    public void periodic() {
        Mode mode;
        if (gripper.hasGamePiece()) {
            if(secondJoint.isBack()) {
                if (vision.backLimelightCentered()) {
                    mode = Mode.HAS_GAME_PIECE_AND_ALLIGNED;
                } else {
                    mode = Mode.HAS_GAME_PIECE;
                }
            }
            else{
                if (vision.frontLimelightCentered()) {
                    mode = Mode.HAS_GAME_PIECE_AND_ALLIGNED;
                } else {
                    mode = Mode.HAS_GAME_PIECE;
                }
            }
        } else {
            if (vision.photonVisionCentered()) {
                mode = Mode.ALLIGNED_TO_GAME_PIECE;
            } else {
                mode = Mode.EMPTY_GRIPPER;
            }
        }
        setMode(mode);
    }

    public void turnOff() {
        setMode(Mode.OFF);
    }

    private void rainbow() {
        // For every pixel
        int rainbowFirstPixelHue = 0;
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            int hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
            // Set the value
            ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 3;
        // Check bounds
        rainbowFirstPixelHue %= 180;
    }

    private void setMode(Mode mode) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, mode.red, mode.green, mode.blue);
        }
        led.setData(ledBuffer);
    }

    private void configureDashboard() {
        namespace.putData("rainbow leds", new InstantCommand(this::rainbow).ignoringDisable(true));
        namespace.putData("turn off", new InstantCommand(this::turnOff).ignoringDisable(true));
    }
}
