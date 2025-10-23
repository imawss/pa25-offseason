package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

public class ModuleConstants {
    public static final Current DRIVE_CURRENT_LIMIT = Amps.of(60);
    public static final Current ANGLE_CURRENT_LIMIT = Amps.of(40);
    
    public static final double DRIVE_P = 20;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.0;
    public static final double DRIVE_IZ = 0.0;

    public static final Constraints DRIVE_CONSTRAINTS = new Constraints(
            MetersPerSecond.of(1000).in(MetersPerSecond),
            MetersPerSecondPerSecond.of(1000).in(MetersPerSecondPerSecond));

    public static final double ANGLE_P = 30;
    public static final double ANGLE_I = 0.0;
    public static final double ANGLE_D = 0.0;
    public static final double ANGLE_IZ = 0.0;

    public static final Constraints ANGLE_CONSTRAINTS = new Constraints(
            RadiansPerSecond.of(1000).in(RadiansPerSecond),
            RadiansPerSecondPerSecond.of(1000).in(RadiansPerSecondPerSecond));

    public static final double DRIVE_GEAR_RATIO = 6.75;
    public static final double STEER_GEAR_RATIO = 150.0 / 7.0;
    public static final double WHEEL_RADIUS_METERS = 0.051;
    public static final Distance WHEEL_RADIUS = null;

}