package frc.robot.services;

import com.spikes2212.dashboard.Namespace;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.RobotMap;

import java.util.function.Supplier;

public class Leds {

    public enum mode {

        EMPTY_GRIPPER(254, 0, 0), ALLIGNED(204, 0, 254), HAS_GAME_PIECE(0, 0, 254),
        HAS_GAME_PIECE_AND_ALLIGNED(0, 254, 0);

        final int red;
        final int green;
        final int blue;
        private int pipeline;

        mode(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }

        int getRed() {
            return this.red;
        }

        int getGreen() {
            return this.green;
        }

        int getBlue() {
            return this.blue;
        }
    }

    public static final int NUMBER_OF_LEDS = 60;
    private static Leds instance;

    private final RootNamespace namespace = new RootNamespace("Leds");

    private final Namespace colorNamespace = namespace.addChild("led colors");
    private Supplier<Integer> red = colorNamespace.addConstantInt("red value", 0);
    private Supplier<Integer> green = colorNamespace.addConstantInt("green value", 0);
    private Supplier<Integer> blue = colorNamespace.addConstantInt("blue value", 0);
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    public static Leds getInstance() {
        if (instance == null) {
            instance = new Leds(new AddressableLED(RobotMap.PWM.LED_PORT),
                    new AddressableLEDBuffer(NUMBER_OF_LEDS));
        }
        return instance;
    }

    private Leds(AddressableLED led, AddressableLEDBuffer ledBuffer) {
        this.led = led;
        this.ledBuffer = new AddressableLEDBuffer(NUMBER_OF_LEDS);
        led.setLength(NUMBER_OF_LEDS);
        configureDashboard();
    }

    private void updateData() {
        int rotateValue = 0; // here in case i decide to work on it

        for (int i = rotateValue; i < ledBuffer.getLength() + rotateValue; i++) {
            if (i % 12 == 3 || i % 12 == 6 || i % 12 == 8 || i % 12 == 11 || i % 12 == 0) {
                ledBuffer.setRGB(i % 60, 0, 0, 0);
            } else {
                ledBuffer.setRGB(i, red.get(), green.get(), blue.get());
            }
        }
        led.setData(ledBuffer);
    }

    private void turnOff() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, 0, 0);
        }
        led.setData(ledBuffer);
    }

    public void configureDashboard() {
        namespace.putRunnable("update color", this::updateData);
        namespace.putRunnable("turn off", this::turnOff);
    }
}
