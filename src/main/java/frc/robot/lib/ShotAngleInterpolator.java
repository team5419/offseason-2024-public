package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.constants.FieldConstants;
import frc.robot.lib.util.AllianceFlipUtil;
import frc.robot.lib.util.Util;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

public class ShotAngleInterpolator {

    private static ShotAngleInterpolator instance = null;
    private InterpolatingDoubleTreeMap armMap, shooterMap;

    /**
     * InterpolatedSuperstructureState - a simple record to hold an interpolated state
     */
    public record InterpolatedSuperstructureState(double interpolatedArmAngle, double interpolatedShooterSpeed) {}

    private ShotAngleInterpolator() {
        armMap = new InterpolatingDoubleTreeMap();
        shooterMap = new InterpolatingDoubleTreeMap();
        clear();
        addInitialInterpolationData();
    }

    public static ShotAngleInterpolator getInstance() {
        if (instance == null) instance = new ShotAngleInterpolator();
        return instance;
    }

    public void addInitialInterpolationData() {
        // TODO: tune
        addArmMeasurement(1.2, 50);
        addArmMeasurement(1.4, 45);
        addArmMeasurement(2.1, 40);
        addArmMeasurement(3.4, 34);
        addArmMeasurement(4.5, 26);
        addArmMeasurement(5.7, 15);

        addShooterMeasurement(1.4, 33.33);
        addShooterMeasurement(4, 66.67);
    }

    public void addArmMeasurement(double distance, double shotAngle) {
        armMap.put(distance, shotAngle);
    }

    public void addShooterMeasurement(double distance, double shotSpeed) {
        shooterMap.put(distance, shotSpeed);
    }

    public void clear() {
        armMap.clear();
        shooterMap.clear();
    }

    public double getInterpolatedArmAngle(double distance) {
        return armMap.get(distance);
    }

    public double getInterpolatedShooterSpeed(double distance) {
        return shooterMap.get(distance);
    }

    public InterpolatedSuperstructureState getInterpolatedSuperstructure(double distance) {
        return new InterpolatedSuperstructureState(
                getInterpolatedArmAngle(distance), getInterpolatedShooterSpeed(distance));
    }

    public InterpolatedSuperstructureState getVelocityCompensatedSuperstructure(double dt) {
        Pose2d newPos = Util.getFuturePosition(dt);
        double distance = PhotonUtils.getDistanceToPose(
                newPos, AllianceFlipUtil.apply(FieldConstants.Speaker.kCenterSpeakerOpeningPose));
        Logger.recordOutput("Interpolation/Future Pose", newPos);
        return getInterpolatedSuperstructure(distance);
    }
}
