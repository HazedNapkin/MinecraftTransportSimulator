package minecrafttransportsimulator.systems.summoning;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Numbers measured over a set of scenarios, and how they compare with the same numbers kept from before: the baseline.
 * Each number is a metric of a scenario.  A metric says which way is better and how much worse than the baseline it
 * may get before that counts as a regression, so a change that makes anything it guards worse is caught, and one that
 * makes things better shows by how much.
 * <p>
 * It's kept as lines of scenario, metric and value, tab separated, so a change to it reads well in a diff.
 */
final class Scorecard {
    /**A number measured for each scenario.**/
    static final class Metric {
        final String name;
        final boolean higherIsBetter;
        /**How much worse than the baseline counts as no worse: this much, or this fraction of the baseline, whichever is more.**/
        final double slack;
        final double slackFraction;
        /**False if it's only kept to look at, like times, which vary from run to run.**/
        final boolean guarded;

        private Metric(String name, boolean higherIsBetter, double slack, double slackFraction, boolean guarded) {
            this.name = name;
            this.higherIsBetter = higherIsBetter;
            this.slack = slack;
            this.slackFraction = slackFraction;
            this.guarded = guarded;
        }
    }

    private final Map<String, Metric> metrics = new LinkedHashMap<>();
    /**Values by scenario, then metric, in the order they were put.**/
    private final Map<String, Map<String, Double>> values = new LinkedHashMap<>();

    /**Adds a metric where lower is better, that may be the slack, or the fraction of the baseline, worse than it.**/
    Scorecard lower(String name, double slack, double slackFraction) {
        metrics.put(name, new Metric(name, false, slack, slackFraction, true));
        return this;
    }

    /**Adds a metric where higher is better.**/
    Scorecard higher(String name, double slack, double slackFraction) {
        metrics.put(name, new Metric(name, true, slack, slackFraction, true));
        return this;
    }

    /**Adds a metric that's only kept to look at, and never counts as a regression.**/
    Scorecard info(String name) {
        metrics.put(name, new Metric(name, false, 0, 0, false));
        return this;
    }

    void put(String scenario, String metric, double value) {
        if (!metrics.containsKey(metric)) {
            throw new IllegalArgumentException("No metric " + metric);
        }
        values.computeIfAbsent(scenario, k -> new LinkedHashMap<>()).put(metric, value);
    }

    /**The value of the metric for the scenario, or NaN if it has none.**/
    double get(String scenario, String metric) {
        Map<String, Double> scenarioValues = values.get(scenario);
        Double value = scenarioValues != null ? scenarioValues.get(metric) : null;
        return value != null ? value : Double.NaN;
    }

    /**The values of the metric over every scenario that has it.**/
    List<Double> getAll(String metric) {
        List<Double> all = new ArrayList<>();
        for (Map.Entry<String, Map<String, Double>> scenario : values.entrySet()) {
            Double value = scenario.getValue().get(metric);
            if (value != null) {
                all.add(value);
            }
        }
        return all;
    }

    /**The scorecard as text, a line per value.**/
    String format() {
        StringBuilder text = new StringBuilder("#scenario\tmetric\tvalue\n");
        for (Map.Entry<String, Map<String, Double>> scenario : values.entrySet()) {
            for (Map.Entry<String, Double> value : scenario.getValue().entrySet()) {
                text.append(scenario.getKey()).append('\t').append(value.getKey()).append('\t').append(format(value.getValue())).append('\n');
            }
        }
        return text.toString();
    }

    private static String format(double value) {
        return value == Math.rint(value) ? String.format(Locale.ROOT, "%.0f", value) : String.format(Locale.ROOT, "%.3f", value);
    }

    /**Reads values kept by {@link #format()}, by scenario then metric.  Returns null if there's nothing to read.**/
    static Map<String, Map<String, Double>> read(InputStream stream) throws IOException {
        if (stream == null) {
            return null;
        }
        Map<String, Map<String, Double>> read = new LinkedHashMap<>();
        try (BufferedReader reader = new BufferedReader(new InputStreamReader(stream, StandardCharsets.UTF_8))) {
            String line;
            while ((line = reader.readLine()) != null) {
                String[] parts = line.split("\t");
                if (!line.startsWith("#") && parts.length == 3) {
                    read.computeIfAbsent(parts[0], k -> new LinkedHashMap<>()).put(parts[1], Double.parseDouble(parts[2]));
                }
            }
        }
        return read;
    }

    /**How this scorecard compares with a baseline.**/
    static final class Comparison {
        /**Guarded values that got worse than their slack allows, or went missing, each as a line to show.**/
        final List<String> regressions = new ArrayList<>();
        /**Guarded values that got better by more than their slack.**/
        final List<String> improvements = new ArrayList<>();
        /**Values the baseline hasn't got.**/
        final List<String> added = new ArrayList<>();
    }

    Comparison compare(Map<String, Map<String, Double>> baseline) {
        Comparison comparison = new Comparison();
        for (Map.Entry<String, Map<String, Double>> scenario : values.entrySet()) {
            Map<String, Double> before = baseline.get(scenario.getKey());
            for (Map.Entry<String, Double> value : scenario.getValue().entrySet()) {
                Metric metric = metrics.get(value.getKey());
                Double was = before != null ? before.get(value.getKey()) : null;
                if (was == null) {
                    comparison.added.add(String.format(Locale.ROOT, "%s %s: %s", scenario.getKey(), metric.name, format(value.getValue())));
                } else if (metric.guarded) {
                    double slack = Math.max(metric.slack, metric.slackFraction * Math.abs(was));
                    double worse = metric.higherIsBetter ? was - value.getValue() : value.getValue() - was;
                    if (Math.abs(worse) > slack + 1e-9) {
                        (worse > 0 ? comparison.regressions : comparison.improvements).add(String.format(Locale.ROOT, "%s %s: %s -> %s", scenario.getKey(), metric.name, format(was), format(value.getValue())));
                    }
                }
            }
        }
        //Anything guarded that's gone is a regression too, or dropping a scenario would hide what it guarded.
        for (Map.Entry<String, Map<String, Double>> scenario : baseline.entrySet()) {
            for (String name : scenario.getValue().keySet()) {
                Metric metric = metrics.get(name);
                if (metric != null && metric.guarded && Double.isNaN(get(scenario.getKey(), name))) {
                    comparison.regressions.add(scenario.getKey() + " " + name + ": no longer measured");
                }
            }
        }
        return comparison;
    }
}
