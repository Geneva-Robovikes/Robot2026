package frc.robot.utils;

import java.io.File;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.lib.BLine.JsonUtils;
import frc.robot.lib.BLine.Path;

public class AutoChooser {
    public SendableChooser<Path> getAutoChooser() {
        File pathFolder = new File(JsonUtils.PROJECT_ROOT, "paths");
        SendableChooser<Path> autoPaths = new SendableChooser<>();

        for(File file : pathFolder.listFiles()) {
            System.out.println(file.getName());
            
            if (file.getName() != "config.json"){ 
                Path autoPath = new Path(file.getName().replace(".json", ""));
                autoPaths.addOption(file.getName().replace(".json", ""), autoPath);
            }
            
        } 
        return autoPaths;
    }
}
