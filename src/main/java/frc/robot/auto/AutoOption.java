package frc.robot.auto;

import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.constants.Constants;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public record AutoOption(
        String name,
        EventLoop autoEventLoop,
        Set<Constants.CompetitionType> competitionTypes
) {
    public static final List<Constants.CompetitionType> defaultCompetitionTypes =
            List.of(Constants.CompetitionType.TESTING);

    public AutoOption(final String name, final EventLoop autoEventLoop) {
        this(name, autoEventLoop, new HashSet<>(defaultCompetitionTypes));
    }

    public AutoOption(
            final String name,
            final EventLoop autoEventLoop,
            final Constants.CompetitionType... competitionTypes
    ) {
        this(
                name,
                autoEventLoop,
                addDefaultCompetitionType(competitionTypes)
        );
    }

    public boolean hasCompetitionType(final Constants.CompetitionType competitionType) {
        return competitionTypes.contains(competitionType);
    }

    private static Set<Constants.CompetitionType> addDefaultCompetitionType(
            final Constants.CompetitionType[] competitionTypes
    ) {
        final HashSet<Constants.CompetitionType> set = new HashSet<>(defaultCompetitionTypes);
        set.addAll(Arrays.asList(competitionTypes));
        return set;
    }
}
