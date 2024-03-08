package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Stream;

public record AutoOption(
        String name,
        Command autoCommand,
        HashSet<Constants.CompetitionType> competitionTypes
) {
    public static final List<Constants.CompetitionType> defaultCompetitionTypes =
            List.of(Constants.CompetitionType.TESTING);

    public AutoOption(final String name, final Command autoCommand) {
        this(name, autoCommand, new HashSet<>(defaultCompetitionTypes));
    }

    public AutoOption(
            final String name,
            final Command autoCommand,
            final Constants.CompetitionType... competitionTypes
    ) {
        this(
                name,
                autoCommand,
                new HashSet<>(Stream.concat(defaultCompetitionTypes.stream(), Arrays.stream(competitionTypes)).toList())
        );
    }

    public boolean hasCompetitionType(final Constants.CompetitionType competitionType) {
        return competitionTypes.contains(competitionType);
    }
}
