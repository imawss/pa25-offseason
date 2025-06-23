package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;

public class RobotConstants {

    public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(4.7244);

    public static Pose2d ORIGIN = new Pose2d();

    public static Pose2d BLUE_START = new Pose2d(
            Meters.of(7.30),
            Meters.of(1),
            Rotation2d.fromDegrees(0));

    public static Pose2d INITIAL_POSE = BLUE_START;

    public static RobotConfig ROBOT_CONFIG;

    static {
        try {
            ROBOT_CONFIG = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}