// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.lib.VirtualSubsystem;
import java.util.List;
import java.util.Optional;

public class Leds extends VirtualSubsystem {
    private static Leds instance;

    public static Leds getInstance() {
        if (instance == null) instance = new Leds();
        return instance;
    }

    // Robot state tracking
    public int loopCycleCount = 0;
    public boolean intaking = false;
    public boolean hasNote = false;
    public boolean autoShoot = false;
    public boolean autoDrive = false;
    public boolean endgameAlert = false;
    public boolean sameBattery = false;
    public boolean autoFinished = false;
    public double autoFinishedTime = 0.0;
    public boolean lowBatteryAlert = false;
    public boolean demoMode = false;
    public boolean isStockpile = false;

    private Optional<Alliance> alliance = Optional.empty();
    private Color allianceColor = Color.kRed;
    private Color secondaryDisabledColor = Color.kBlack;
    private boolean lastEnabledAuto = false;
    private double lastEnabledTime = 0.0;
    private boolean estopped = false;

    // LED IO
    private final AddressableLED leds;
    private final AddressableLEDBuffer buffer;
    private final Notifier loadingNotifier;

    // Constants
    private static final boolean prideLeds = false;
    private static final int minLoopCycleCount = 10;
    private static final int length = 80;
    private static final double strobeDuration = 0.1;
    private static final double breathDuration = 1.0;
    private static final double rainbowCycleLength = 25.0;
    private static final double rainbowDuration = 0.25;
    private static final double waveExponent = 0.4;
    private static final double waveFastCycleLength = 25.0;
    private static final double waveFastDuration = 0.25;
    private static final double waveAllianceCycleLength = 15.0;
    private static final double waveAllianceDuration = 2.0;
    private static final double autoFadeTime = 2.5; // 3s nominal
    private static final double autoFadeMaxTime = 5.0; // Return to normal

    private static List<Color> prideColors = List.of(
            Color.kBlack,
            Color.kRed,
            Color.kOrangeRed,
            Color.kYellow,
            Color.kGreen,
            Color.kBlue,
            Color.kPurple,
            Color.kBlack,
            new Color(0.15, 0.3, 1.0),
            Color.kDeepPink,
            Color.kWhite,
            Color.kDeepPink,
            new Color(0.15, 0.3, 1.0));

    private Leds() {
        leds = new AddressableLED(9);
        buffer = new AddressableLEDBuffer(length);
        leds.setLength(length);
        leds.setData(buffer);
        leds.start();
        loadingNotifier = new Notifier(() -> {
            synchronized (this) {
                breath(Color.kWhite, Color.kBlack, System.currentTimeMillis() / 1000.0);
                leds.setData(buffer);
            }
        });
        loadingNotifier.startPeriodic(0.02);
    }

    private void robotDisabledLEDTasks() {
        // performs led logic and returns if a condition is met based on priority
        if (sameBattery) {
            // same battery aler
            breath(Color.kRed, Color.kBlack);
            return; // perhaps you dont want this?
        }
        if (lastEnabledAuto && Timer.getFPGATimestamp() - lastEnabledTime < autoFadeMaxTime) {
            // Auto fade
            solid(1.0 - ((Timer.getFPGATimestamp() - lastEnabledTime) / autoFadeTime), Color.kGreen);
            return;
        }
        if (lowBatteryAlert) {
            // Low battery
            solid(Color.kOrangeRed);
            return;
        }
        if (prideLeds) {
            // Pride stripes
            stripes(prideColors, 3, 5.0);
            return;
        }
        breath(secondaryDisabledColor, allianceColor); // if none of the above conditions are met
    }

    private void robotEnabledLEDTasks() {
        // performs led logic and returns if a condition is met based on priority
        if (endgameAlert) {
            strobe(Color.kRed, Color.kGold, strobeDuration);
            return;
        }
        if (autoDrive || autoShoot) {
            rainbow(rainbowCycleLength, rainbowDuration);
            return;
        }
        if (hasNote && isStockpile) {
            solid(Color.kDarkBlue);
            return;
        }
        if (hasNote && !isStockpile) {
            solid(Color.kDeepPink);
            return;
        }
        // breath(secondaryDisabledColor, allianceColor); // if none of the above conditions are met, not sure if this
        // is what you guys want it to do
    }

    private void autonomousLEDTasks() {
        wave(Color.kGold, Color.kRed, waveFastCycleLength, waveFastDuration);
        if (autoFinished) {
            double fullTime = (double) length / waveFastCycleLength * waveFastDuration;
            solid((Timer.getFPGATimestamp() - autoFinishedTime) / fullTime, Color.kGreen);
        }
    }

    public synchronized void periodic() {
        // Update alliance color
        if (DriverStation.isFMSAttached()) {
            alliance = DriverStation.getAlliance();
            allianceColor = alliance.map(alliance -> alliance == Alliance.Blue ? Color.kBlue : Color.kRed)
                    .orElse(Color.kGold);
            secondaryDisabledColor = alliance.isPresent() ? Color.kBlack : Color.kDarkBlue;
        }

        // Update auto state
        if (DriverStation.isDisabled()) {
            autoFinished = false;
        } else {
            lastEnabledAuto = DriverStation.isAutonomous();
            lastEnabledTime = Timer.getFPGATimestamp();
        }

        // Update estop state
        if (DriverStation.isEStopped()) estopped = true;

        if (!DriverStation.isDSAttached())
            wave(Color.kGold, Color.kBlack, waveAllianceCycleLength, waveAllianceDuration);

        // Exit during initial cycles
        loopCycleCount += 1;
        if (loopCycleCount < minLoopCycleCount) {
            return;
        }

        // Stop loading notifier if running
        loadingNotifier.stop();

        // Select LED mode
        solid(Color.kBlack); // Default to off
        if (estopped) {
            solid(Color.kRed);
        } else if (DriverStation.isDisabled()) {
            robotDisabledLEDTasks();
        } else if (DriverStation.isAutonomous()) {
            autonomousLEDTasks();
        } else { // Enabled
            robotEnabledLEDTasks();
        }

        // Update LEDs
        leds.setData(buffer);
    }

    private void solid(Color color) {
        if (color != null) {
            for (int i = 0; i < length; i++) {
                buffer.setLED(i, color);
            }
        }
    }

    private void solid(double percent, Color color) {
        for (int i = 0; i < MathUtil.clamp(length * percent, 0, length); i++) {
            buffer.setLED(i, color);
        }
    }

    private void strobe(Color c1, Color c2, double duration) {
        boolean c1On = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
        solid(c1On ? c1 : c2);
    }

    private void strobe(Color color, double duration) {
        strobe(color, Color.kBlack, duration);
    }

    private void breath(Color c1, Color c2) {
        breath(c1, c2, Timer.getFPGATimestamp());
    }

    private void breath(Color c1, Color c2, double timestamp) {
        double x = ((timestamp % breathDuration) / breathDuration) * 2.0 * Math.PI;
        double ratio = (Math.sin(x) + 1.0) / 2.0;
        double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
        double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
        double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
        solid(new Color(red, green, blue));
    }

    private void rainbow(double cycleLength, double duration) {
        double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
        double xDiffPerLed = 180.0 / cycleLength;
        for (int i = 0; i < length; i++) {
            x += xDiffPerLed;
            x %= 180.0;
            buffer.setHSV(i, (int) x, 255, 255);
        }
    }

    private void wave(Color c1, Color c2, double cycleLength, double duration) {
        double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
        double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
        for (int i = 0; i < length; i++) {
            x += xDiffPerLed;
            double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
            if (Double.isNaN(ratio)) {
                ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
            }
            if (Double.isNaN(ratio)) {
                ratio = 0.5;
            }
            double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
            double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
            double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
            buffer.setLED(i, new Color(red, green, blue));
        }
    }

    private void stripes(List<Color> colors, int stripeLength, double duration) {
        int offset = (int) (Timer.getFPGATimestamp() % duration / duration * stripeLength * colors.size());
        for (int i = 0; i < length; i++) {
            int colorIndex = (int) (Math.floor((double) (i - offset) / stripeLength) + colors.size()) % colors.size();
            colorIndex = colors.size() - 1 - colorIndex;
            buffer.setLED(i, colors.get(colorIndex));
        }
    }
}
