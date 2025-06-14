package frc.robot.constants;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class AutoConstants {

    public static final double DRIVE_P = 5;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.0;
    public static final double DRIVE_IZ = 0.0;

    public static final PIDConstants DRIVE_PID = new PIDConstants(DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_IZ);

    public static final Constraints DRIVE_CONSTRAINTS = new Constraints(
            MetersPerSecond.of(1000).in(MetersPerSecond),
            MetersPerSecondPerSecond.of(1000).in(MetersPerSecondPerSecond));

    public static final double ANGLE_P = 5;
    public static final double ANGLE_I = 0.0;
    public static final double ANGLE_D = 0.0;
    public static final double ANGLE_IZ = 0.0;

    public static final PIDConstants ANGLE_PID = new PIDConstants(ANGLE_P, ANGLE_I, ANGLE_D, ANGLE_IZ);

    public static final Constraints ANGLE_CONSTRAINTS = new Constraints(
            DegreesPerSecond.of(1000).in(DegreesPerSecond),
            DegreesPerSecondPerSecond.of(1000).in(DegreesPerSecondPerSecond));
}