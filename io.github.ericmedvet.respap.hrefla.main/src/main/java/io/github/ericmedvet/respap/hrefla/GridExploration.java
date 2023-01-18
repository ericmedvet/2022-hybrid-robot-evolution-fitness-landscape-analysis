package io.github.ericmedvet.respap.hrefla;

import com.beust.jcommander.JCommander;
import com.beust.jcommander.Parameter;
import com.beust.jcommander.ParameterException;
import io.github.ericmedvet.jgea.experimenter.InvertibleMapper;
import io.github.ericmedvet.jnb.core.NamedBuilder;
import io.github.ericmedvet.mrsim2d.core.EmbodiedAgent;
import io.github.ericmedvet.mrsim2d.core.engine.Engine;
import io.github.ericmedvet.mrsim2d.core.tasks.Task;
import io.github.ericmedvet.mrsim2d.core.util.DoubleRange;
import io.github.ericmedvet.robotevo2d.main.PreparedNamedBuilder;
import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVPrinter;

import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;
import java.util.ServiceLoader;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.logging.Logger;
import java.util.stream.IntStream;

/**
 * @author "Eric Medvet" on 2023/01/18 for 2022-hybrid-robot-evolution-fitness-landscape-analysis
 */
public class GridExploration implements Runnable {

  private final static Logger L = Logger.getLogger(Starter.class.getName());
  private final static String LEGGED_OPEN_2_MAPPER = """
      s.a.numLeggedHybridRobot(
        trunkLength = 10;
        trunkMass = 1;
        legs = 4 * [
          s.a.l.leg(downConnector = soft; legChunks = 2 * [s.a.l.legChunk()])
        ];
        function = s.f.groupedSin(
          size = 3;
          p = s.range(min = -1.57; max = 1.57);
          f = s.range(min = 1.0; max = 1.0);
          a = s.range(min = 0.5; max = 0.5);
          b = s.range(min = 0.0; max = 0.0);
          s = s.range(min = 0.0; max = 0.0)
        )
      )
      """;
  private final static String TASK = "s.task.locomotion(terrain = s.t.steppy(chunkW = 2; chunkH = 0.25))";
  private final static String C_FUNCTION = "s.task.locomotion.xVelocity()";
  private final static String MAPPER_TEMPLATE = "er.m.parametrizedHomoBrains(target=%TARGET%)";
  private final static DoubleRange GENE_DOMAIN = DoubleRange.SYMMETRIC_UNIT;
  @Parameter(
      names = {"--outputFile", "-of"},
      description = "Path of the file for the output"
  )
  public String outputFile = "";
  @Parameter(
      names = {"--nOfValuesPerDimension", "-n"},
      description = "Number of values per each dimension of the genotype space"
  )
  public int nOfValuesPerDimension = 1;

  private static <T> List<T> append(List<T> ts, T t) {
    List<T> newTs = new ArrayList<>(ts);
    newTs.add(t);
    return newTs;
  }

  public static void main(String[] args) {
    try {
      GridExploration gridExploration = new GridExploration();
      JCommander.newBuilder()
          .addObject(gridExploration)
          .build()
          .parse(args);
      gridExploration.run();
    } catch (ParameterException e) {
      e.usage();
      L.severe(String.format("Cannot read command line options: %s", e));
      System.exit(-1);
    } catch (RuntimeException e) {
      L.severe(e.getClass().getSimpleName() + ": " + e.getMessage());
      System.exit(-1);
    }
  }

  private static <T> List<List<T>> product(List<List<T>> lists, List<T> values) {
    return values.stream()
        .map(v -> lists.stream()
            .map(l -> append(l, v))
            .toList()
        )
        .flatMap(List::stream)
        .toList();
  }

  public void run() {
    NamedBuilder<?> nb = PreparedNamedBuilder.get();
    Supplier<Engine> engineSupplier = () -> ServiceLoader.load(Engine.class)
        .findFirst()
        .orElseThrow(() -> new RuntimeException("Cannot instantiate an engine"));
    @SuppressWarnings("unchecked") Function<Object, Double> cFunction = (Function<Object, Double>) nb.build(C_FUNCTION);
    @SuppressWarnings("unchecked") Task<Supplier<EmbodiedAgent>, ?> task =
        (Task<Supplier<EmbodiedAgent>, ?>) nb.build(
            TASK);
    @SuppressWarnings("unchecked") InvertibleMapper<List<Double>, Supplier<EmbodiedAgent>> mapper =
        (InvertibleMapper<List<Double>, Supplier<EmbodiedAgent>>) nb.build(
            MAPPER_TEMPLATE.replace("%TARGET%", LEGGED_OPEN_2_MAPPER));
    int p = mapper.exampleInput().size();
    List<Double> points = IntStream.range(0, nOfValuesPerDimension)
        .mapToDouble(i -> (double) i / (double) (nOfValuesPerDimension - 1))
        .map(GENE_DOMAIN::denormalize)
        .boxed()
        .toList();
    System.out.printf("Will use %d values per dimension, with %d dimensions.%n", points.size(), p);
    List<List<Double>> genotypes = points.stream().map(List::of).toList();
    for (int i = 1; i < p; i = i + 1) {
      genotypes = product(genotypes, points);
    }
    System.out.printf("Will use %d genotypes.%n", genotypes.size());
    //do runs
    ExecutorService executorService = Executors.newFixedThreadPool(Runtime.getRuntime().availableProcessors());
    record Outcome(List<Double> genotype, double q) {}
    List<Future<Outcome>> futures = genotypes.stream()
        .map(g -> executorService.submit(() -> new Outcome(
                g,
                cFunction.apply(task.run(mapper.apply(g), engineSupplier.get()))
            )
        ))
        .toList();
    //save results
    CSVPrinter printer;
    try {
      printer = new org.apache.commons.csv.CSVPrinter(
          new PrintStream(outputFile),
          CSVFormat.Builder.create().setDelimiter(";").build()
      );
      List<String> names = new ArrayList<>(List.of("q"));
      names.addAll(IntStream.range(0, p).mapToObj("g%d"::formatted).toList());
      printer.printRecord(names);
    } catch (IOException e) {
      L.severe("Cannot open output file: %s".formatted(e));
      executorService.shutdownNow();
      return;
    }
    AtomicInteger counter = new AtomicInteger(0);
    futures.forEach(f -> {
      try {
        Outcome outcome = f.get();
        System.out.printf(
            "Outcome %d/%d for found: %6.3f%n", counter.incrementAndGet(),
            futures.size(),
            outcome.q()
        );
        List<Object> values = new ArrayList<>(List.of(outcome.q()));
        values.addAll(outcome.genotype);
        printer.printRecord(values);
      } catch (InterruptedException | ExecutionException e) {
        L.severe("Cannot get result due to: %s".formatted(e));
      } catch (IOException e) {
        L.severe("Cannot print result due to: %s".formatted(e));
      }
    });
    executorService.shutdown();
  }
}
