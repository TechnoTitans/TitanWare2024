package frc.robot.auto;

import com.choreo.lib.ChoreoTrajectoryState;
import com.google.gson.Gson;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

public class AutoLoader {
    private static final Gson gson = new Gson();

    private AutoLoader() {}

    /**
     * Load a trajectory from the deploy directory. This expects .traj files to be placed in
     * src/main/deploy/choreo/[trajName].traj .
     *
     * @param trajName the path name in Choreo, which matches the file name in the deploy directory.
     *     Do not include ".traj" here.
     * @return the loaded trajectory, or null if the trajectory could not be loaded.
     */
    public static TitanTrajectory getTrajectory(final String trajName) {
        final File traj_dir = new File(Filesystem.getDeployDirectory(), "choreo");
        final File traj_file = new File(traj_dir, trajName + ".traj");

        return loadFile(traj_file);
    }

    /**
     * Loads the split parts of the specified trajectory. Fails and returns null if any of the parts
     * could not be loaded.
     *
     * <p>This method determines the number of parts to load by counting the files that match the
     * pattern "trajName.X.traj", where X is a string of digits. Let this count be N. It then attempts
     * to load "trajName.1.traj" through "trajName.N.traj", consecutively counting up. If any of these
     * files cannot be loaded, the method returns null.
     *
     * @param trajName The path name in Choreo for this trajectory.
     * @return The ArrayList of segments, in order, or null.
     */
    public static ArrayList<TitanTrajectory> getTrajectoryGroup(final String trajName) {
        // Count files matching the pattern for split parts.
        final File traj_dir = new File(Filesystem.getDeployDirectory(), "choreo");
        final File[] files =
                traj_dir.listFiles((file) -> file.getName().matches(trajName + "\\.\\d+\\.traj"));
        final int segmentCount = files.length;
        // Try to load the segments.
        final ArrayList<TitanTrajectory> trajs = new ArrayList<>();
        for (int i = 1; i <= segmentCount; ++i) {
            final File traj = new File(traj_dir, String.format("%s.%d.traj", trajName, i));
            final TitanTrajectory trajectory = loadFile(traj);
            if (trajectory == null) {
                DriverStation.reportError("Missing segments for path group " + trajName, false);
                throw new RuntimeException("Missing segments for path group " + trajName);
            }
            trajs.add(trajectory);
        }

        return trajs;
    }

    private static TitanTrajectory loadFile(File path) {
        try {
            final BufferedReader reader = new BufferedReader(new FileReader(path));

            final StringBuilder fileContentBuilder = new StringBuilder();
            String line;
            while ((line = reader.readLine()) != null) {
                fileContentBuilder.append(line);
            }

            final String fileContent = fileContentBuilder.toString();
            final JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

            final List<ChoreoTrajectoryState> choreoTrajectoryStates = new ArrayList<>();
            for (var trajectoryState : (JSONArray) json.get("samples")) {
                choreoTrajectoryStates.add(gson.fromJson(trajectoryState.toString(), ChoreoTrajectoryState.class));
            }

            final List<TitanTrajectory.TitanMarker> eventMarkers = new ArrayList<>();
            for (var markerJson : (JSONArray) json.get("eventMarkers")) {
                eventMarkers.add(TitanTrajectory.TitanMarker.fromJson((JSONObject) markerJson));
            }

            return new TitanTrajectory(choreoTrajectoryStates, eventMarkers);
        } catch (final Exception ex) {
            DriverStation.reportError(ex.getMessage(), ex.getStackTrace());
        }
        return null;
    }
}
