package frc.robot.utils.logging;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.struct.Struct;
import org.littletonrobotics.junction.LogTable;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

public class LogUtils {
    public static final double MICRO_TO_MILLI = 1d / 1000;
    public static double microsecondsToMilliseconds(final double microseconds) {
        return microseconds * MICRO_TO_MILLI;
    }

    public static void serializePhotonPipelineResult(
            final LogTable logTable,
            final String prefix,
            final PhotonPipelineResult photonPipelineResult
    ) {
        if (photonPipelineResult == null) {
            logTable.put(prefix + "/IsPresent", false);
            return;
        } else {
            logTable.put(prefix + "/IsPresent", false);
        }

        final int targetsUsedSize = photonPipelineResult.targets.size();
        logTable.put(prefix + "/Targets/Size", targetsUsedSize);

        for (int i = 0; i < targetsUsedSize; i++) {
            final PhotonTrackedTarget trackedTarget = photonPipelineResult.targets.get(i);
            final Packet packet = new Packet(PhotonTrackedTarget.serde.getMaxByteSize());

            PhotonTrackedTarget.serde.pack(packet, trackedTarget);
            LogUtils.serializePhotonVisionPacket(logTable, prefix + "/Targets/Packet" + i, packet);
        }

        logTable.put(prefix + "/LatencyMillis", photonPipelineResult.getLatencyMillis());
        logTable.put(prefix + "/TimestampSeconds", photonPipelineResult.getTimestampSeconds());

        LogUtils.serializeMultiTargetPNPResult(
                logTable,
                prefix + "/MultiTagResult",
                photonPipelineResult.getMultiTagResult()
        );
    }

    public static PhotonPipelineResult deserializePhotonPipelineResult(final LogTable logTable, final String prefix) {
        if (logTable.get(prefix + "/IsPresent", false)) {
            return null;
        }

        final long targetsUsedSize = logTable.get(prefix + "/Targets/Size", 0);
        final List<PhotonTrackedTarget> trackedTargets = new ArrayList<>();
        for (int i = 0; i < targetsUsedSize; i++) {
            final Packet packet = LogUtils.deserializePhotonVisionPacket(
                    logTable, prefix + "/Targets/Packet" + i
            );

            final PhotonTrackedTarget trackedTarget = PhotonTrackedTarget.serde.unpack(packet);
            trackedTargets.add(trackedTarget);
        }

        final double latencyMillis = logTable.get(prefix + "/LatencyMillis", 0);
        final double timestampSeconds = logTable.get(prefix + "/TimestampSeconds", -1);

        final MultiTargetPNPResult multiTargetPNPResult =
                LogUtils.deserializeMultiTargetPNPResult(logTable, prefix + "/MultiTagResult");

        final PhotonPipelineResult result = new PhotonPipelineResult(
                latencyMillis,
                trackedTargets,
                multiTargetPNPResult
        );
        result.setTimestampSeconds(timestampSeconds);

        return result;
    }

    public static final PNPResultStruct PNPResultStruct = new PNPResultStruct();
    public static class PNPResultStruct implements Struct<PNPResult> {
        @Override
        public Class<PNPResult> getTypeClass() {
            return PNPResult.class;
        }

        @Override
        public String getTypeString() {
            return "struct:PNPResult";
        }

        @Override
        public int getSize() {
            return kSizeBool
                    + Transform3d.struct.getSize()
                    + kSizeDouble
                    + Transform3d.struct.getSize()
                    + kSizeDouble
                    + kSizeDouble;
        }

        @Override
        public String getSchema() {
            return "bool isPresent;Transform3d best;double bestReprojErr;Transform3d alt;double altReprojErr;double ambiguity";
        }

        @Override
        public Struct<?>[] getNested() {
            return new Struct<?>[]{Transform3d.struct};
        }

        @Override
        public PNPResult unpack(final ByteBuffer bb) {
            final boolean isPresent = bb.get() != 0;
            if (!isPresent) {
                return new PNPResult();
            }

            final Transform3d best = Transform3d.struct.unpack(bb);
            final double bestReprojErr = bb.getDouble();
            final Transform3d alt = Transform3d.struct.unpack(bb);
            final double altReprojErr = bb.getDouble();
            final double ambiguity = bb.getDouble();

            return new PNPResult(best, alt, ambiguity, bestReprojErr, altReprojErr);
        }

        @Override
        public void pack(final ByteBuffer bb, final PNPResult value) {
            bb.put((byte)(value.isPresent ? 1 : 0));
            Transform3d.struct.pack(bb, value.best);
            bb.putDouble(value.bestReprojErr);
            Transform3d.struct.pack(bb, value.alt);
            bb.putDouble(value.altReprojErr);
            bb.putDouble(value.ambiguity);
        }
    }

    public static void serializeMultiTargetPNPResult(
            final LogTable logTable,
            final String prefix,
            final MultiTargetPNPResult multiTargetPNPResult
    ) {
        logTable.put(prefix + "/PNPResult", PNPResultStruct, multiTargetPNPResult.estimatedPose);

        final int nFiducialsUsed = multiTargetPNPResult.fiducialIDsUsed.size();
        final int[] fiducialIDsUsed = new int[nFiducialsUsed];
        for (int i = 0; i < nFiducialsUsed; i++) {
            fiducialIDsUsed[i] = multiTargetPNPResult.fiducialIDsUsed.get(i);
        }

        logTable.put(prefix + "/FiducialIDsUsed", fiducialIDsUsed);
    }

    private static final PNPResult DefaultPNPResult = new PNPResult();
    public static MultiTargetPNPResult deserializeMultiTargetPNPResult(final LogTable logTable, final String prefix) {
        final PNPResult pnpResult = logTable.get(prefix + "/PNPResult", PNPResultStruct, DefaultPNPResult);

        final int[] fiducialIDsUsed = logTable.get(prefix + "/FiducialIDsUsed", new int[0]);
        final List<Integer> fiducialIDs = new ArrayList<>();
        for (final int id : fiducialIDsUsed) {
            fiducialIDs.add(id);
        }

        return new MultiTargetPNPResult(pnpResult, fiducialIDs);
    }

    public static void serializePhotonVisionPacket(final LogTable logTable, final String prefix, final Packet packet) {
        logTable.put(prefix + "/packetData", packet.getData());
    }

    public static Packet deserializePhotonVisionPacket(final LogTable logTable, final String prefix) {
        return new Packet(logTable.get(prefix + "/packetData", new byte[] {}));
    }

    /**
     * Since PathPlanner trajectories seem to have >1k (even up to 2k) states in each trajectory,
     * AdvantageScope struggles to log all the states (poses) and it lags out the interface
     * <p>
     * This class reduces the number of states down to a maximum of {@link LoggableTrajectory#MAX_STATES} so that
     * it can be properly logged
     * <p>
     * Do <b>NOT</b> rely on this class to provide accurate states - this should ONLY be used for logging purposes
     * @see Trajectory
     */
    @SuppressWarnings("unused")
    public static class LoggableTrajectory extends Trajectory {
        public static final double TIME_ACCURACY = 0.1;
        public static final int MAX_STATES = 25;

        public LoggableTrajectory(final List<State> states) {
            super(states);
        }

        /**
         * Create a new {@link LoggableTrajectory} from a {@link Trajectory} object.
         * @param trajectory the {@link Trajectory} to create from
         * @return the new {@link LoggableTrajectory}
         * @see Trajectory
         */
        public static LoggableTrajectory fromTrajectory(final Trajectory trajectory) {
            return new LoggableTrajectory(trajectory.getStates());
        }

        /**
         * Return a reduced number of states (a maximum of {@link LoggableTrajectory#MAX_STATES})
         * in this {@link LoggableTrajectory}.
         * <p>
         * To get the entire {@link List} of {@link State}s,
         * use {@link LoggableTrajectory#getAllStates()}
         * <p>
         * Do <b>NOT</b> rely on this to report an accurate list of states
         * @return the reduced {@link List} of {@link State}s
         * @see Trajectory#getStates()
         */
        @Override
        public List<State> getStates() {
            final double totalTime = this.getTotalTimeSeconds();
            final double timeDiffPerReportedState = Math.max(totalTime / MAX_STATES, TIME_ACCURACY);

            final List<State> allStates = super.getStates();
            final List<State> reportedStates = new ArrayList<>(Math.min(allStates.size(), MAX_STATES));

            // return empty list when there are no states, fast-path if there's only one state
            if (allStates.isEmpty()) {
                return List.of();
            } else if (allStates.size() == 1) {
                return List.of(allStates.get(0));
            }

            State lastState = allStates.get(0);
            for (double t = allStates.get(1).timeSeconds; t <= totalTime; t += timeDiffPerReportedState) {
                final State nextState = this.sample(t);
                if (nextState != lastState) {
                    reportedStates.add(nextState);
                    lastState = nextState;
                }
            }

            return reportedStates;
        }

        /**
         * Returns all states without any reduction. Equivalent to {@link Trajectory#getStates()}
         * @return the list of states
         */
        public List<State> getAllStates() {
            return super.getStates();
        }
    }
}
