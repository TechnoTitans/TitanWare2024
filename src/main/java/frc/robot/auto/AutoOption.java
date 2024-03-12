package frc.robot.auto;

import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.constants.Constants;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.stream.Stream;

public record AutoOption(
        String name,
        EventLoop autoEventLoop,
        HashSet<Constants.CompetitionType> competitionTypes
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
                new HashSet<>(Stream.concat(defaultCompetitionTypes.stream(), Arrays.stream(competitionTypes)).toList())
        );
    }

    public boolean hasCompetitionType(final Constants.CompetitionType competitionType) {
        return competitionTypes.contains(competitionType);
    }
}
