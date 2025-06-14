package frc.robot.utils;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Logger extends DogLog {
    public static void log(String key, SendableChooser<?> chooser){
        SmartDashboard.putData(key, chooser);
    }
}
