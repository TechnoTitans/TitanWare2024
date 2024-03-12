package frc.robot.auto;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import org.json.simple.JSONObject;

import java.util.List;

public class TitanTrajectory extends ChoreoTrajectory {
    public record TitanMarker(double timestamp) {

        public static TitanMarker fromJson(final JSONObject markerJson) {
                final double timestamp = ((Number) markerJson.get("timestamp")).doubleValue();
                return new TitanMarker(timestamp);
            }
        }

    public final List<TitanMarker> markers;

    public TitanTrajectory(final List<ChoreoTrajectoryState> samples, final List<TitanMarker> markers) {
        super(samples);
        this.markers = markers;
    }

    public List<TitanMarker> getMarkers() {
        return markers;
    }
}
