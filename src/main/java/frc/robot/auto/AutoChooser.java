package frc.robot.auto;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import frc.robot.constants.Constants;
import frc.robot.utils.closeables.ToClose;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

public class AutoChooser<I, V extends AutoOption> implements AutoCloseable, LoggedDashboardInput {
    private final String ntTableName;

    private final StringArrayPublisher autoPublisher;
    private final StringSubscriber selectedAutoSubscriber;
    private final LinkedHashMap<String, V> autoMap;
    private final HashSet<V> ignoredSet;
    private final LoggableInputs inputs = new LoggableInputs() {
        @Override
        public void toLog(LogTable table) {
            table.put(ntTableName, selectedAuto);
        }

        @Override
        public void fromLog(LogTable table) {
            selectedAuto = table.get(ntTableName, selectedAuto);
        }
    };

    private final V defaultAuto;

    private String selectedAuto;

    public AutoChooser(final V defaultAuto) {
        this.ntTableName = Constants.NetworkTables.AUTO_TABLE;
        this.ignoredSet = new HashSet<>();
        this.defaultAuto = defaultAuto;
        this.selectedAuto = defaultAuto != null
                ? defaultAuto.name()
                : String.format("DoNothingAuto-%s", this);

        final NetworkTable ntTable = NetworkTableInstance.getDefault().getTable(ntTableName);
        this.autoPublisher = ntTable.getStringArrayTopic(Constants.NetworkTables.AUTO_PUBLISHER).publish();
        this.selectedAutoSubscriber = ntTable.getStringTopic(Constants.NetworkTables.AUTO_SELECTED_SUBSCRIBER).subscribe(selectedAuto);

        final Map<String, V> initialAutoMap = defaultAuto != null ? Map.of(selectedAuto, defaultAuto) : Map.of();
        this.autoMap = new LinkedHashMap<>(initialAutoMap);

        Logger.registerDashboardInput(this);
        ToClose.add(this);
    }

    private boolean shouldIgnoreOption(final V object) {
        return ignoredSet.contains(object) || !object.hasCompetitionType(Constants.CURRENT_COMPETITION_TYPE);
    }

    public void addOption(final String name, final V object) {
        if (shouldIgnoreOption(object)) {
            ignoredSet.add(object);
        } else {
            autoMap.put(name, object);
            autoPublisher.set(autoMap.keySet().toArray(String[]::new));
        }
    }

    /**
     * Adds an {@link AutoOption} object to the by using the name of the option
     * @param object the {@link AutoOption} or V
     * @see AutoOption
     * @see AutoChooser#addOption(String, AutoOption)
     */
    public void addAutoOption(final V object) {
        addOption(object.name(), object);
    }

    public V getSelected() {
        final String selectedAutoName = selectedAutoSubscriber.get();
        return selectedAutoName != null ? autoMap.get(selectedAutoName) : defaultAuto;
    }

    public void addOptionsIfNotPresent(
            final Function<V, String> nameFunction, final Function<I, V> objectFunction, final List<I> inputs
    ) {
        for (final I input : inputs) {
            final V computedObject = objectFunction.apply(input);
            if (!autoMap.containsValue(computedObject)) {
                addOption(nameFunction.apply(computedObject), computedObject);
            }
        }
    }

    /**
     * Get a copy of the currently registered {@link V}s
     * @return a {@link List} of currently registered {@link V}s
     */
    public List<V> getRegisteredOptions() {
        return autoMap.values().stream().toList();
    }

    @Override
    public void close() {
        selectedAutoSubscriber.close();
        autoPublisher.close();
    }

    @Override
    public void periodic() {
        if (!Logger.hasReplaySource()) {
            selectedAuto = selectedAutoSubscriber.get();
        }
        Logger.processInputs(prefix, inputs);
    }
}
