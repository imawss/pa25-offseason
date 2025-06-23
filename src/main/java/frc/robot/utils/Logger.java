package frc.robot.utils;

import com.pathplanner.lib.config.RobotConfig;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.RobotConstants;

public class Logger extends DogLog {
    public static void log(String key, SendableChooser<?> chooser){
        SmartDashboard.putData(key, chooser);
    }
}
