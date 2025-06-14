package frc.robot.constants;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;

public class DrivetrainConstants {

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(Meters.of(+0.256), Meters.of(+0.256)),
            new Translation2d(Meters.of(+0.256), Meters.of(-0.256)),
            new Translation2d(Meters.of(-0.256), Meters.of(+0.256)),
            new Translation2d(Meters.of(-0.256), Meters.of(-0.256)));

    public static final DriveTrainSimulationConfig DRIVETRAIN_CONFIG = DriveTrainSimulationConfig.Default()
            .withBumperSize(Meters.of(0.710), Meters.of(0.710))
            .withRobotMass(Kilograms.of(52.5))
            .withGyro(COTS.ofPigeon2())
            .withCustomModuleTranslations(KINEMATICS.getModules())
            .withSwerveModule(COTS.ofMark4i(
                    DCMotor.getKrakenX60(1),
                    DCMotor.getKrakenX60(1),
                    1.1,
                    2));
}