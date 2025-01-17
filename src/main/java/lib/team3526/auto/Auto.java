package lib.team3526.auto;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import edu.wpi.first.wpilibj.Filesystem;

public class Auto {
    public class utils {
        /**
         * Get the names of all the paths in the pathplanner/paths directory
         * @return a list of all the path names
         */
        List<String> getAllPathNames() {
            File[] pathFiles = new File(Filesystem.getDeployDirectory(), "pathplanner/paths").listFiles();

            if (pathFiles == null) return new ArrayList<String>();

            return Stream.of(pathFiles)
                .filter(file -> !file.isDirectory())
                .map(File::getName)
                .filter(name -> name.endsWith(".path"))
                .map(name -> name.substring(0, name.lastIndexOf(".")))
                .collect(Collectors.toList());
        }
    }
}
