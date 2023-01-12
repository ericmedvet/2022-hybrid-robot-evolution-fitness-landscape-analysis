/*
 * Copyright 2023 eric
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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
import org.apache.commons.csv.CSVParser;
import org.apache.commons.csv.CSVPrinter;

import java.io.*;
import java.util.*;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.logging.Logger;
import java.util.random.RandomGenerator;
import java.util.stream.IntStream;

public class Starter implements Runnable {

  private final static Logger L = Logger.getLogger(Starter.class.getName());

  private final static String VSR_TARGET_TEMPLATE = """
      s.a.centralizedNumGridVSR(
        body = s.a.vsr.gridBody(
          sensorizingFunction = s.a.vsr.sf.directional(
            headSensors = [s.s.sin(f = 0); s.s.d(a = -30; r = 10)];
            nSensors = [s.s.ar(); s.s.rv(a = 0); s.s.rv(a = 90)];
            sSensors = [s.s.d(a = -90)]
          );
          shape = %SHAPE%
        );
        function = %FUNCTION%
      )
      """;

  private final static String C_FUNCTION = "s.task.locomotion.xVelocity()";

  private final static String MAPPER_TEMPLATE = "er.m.parametrizedHomoBrains(target=%TARGET%)";

  @Parameter(
      names = {"--inputFile", "-if"},
      description = "Path of the input file with the results of the evolution"
  )
  public String inputFile = "";
  @Parameter(
      names = {"--outputFile", "-of"},
      description = "Path of the file for the output"
  )
  public String outputFile = "";
  @Parameter(
      names = {"--iterations", "-i"},
      description = "Iteration index to extract best individuals"
  )
  public List<Integer> iterations = List.of();
  @Parameter(
      names = {"--nOfDestinations", "-nd"},
      description = "Number of target genotypes for each best"
  )
  public int nOfDestinations = 1;
  @Parameter(
      names = {"--destinationDistance", "-d"},
      description = "Euclidean distance of the target"
  )
  public double destinationDistance = 0.5;
  @Parameter(
      names = {"--nOfSteps", "-np"},
      description = "Number of points for each best-to-target section"
  )
  public int nOfSteps = 10;
  @Parameter(
      names = {"--task"},
      description = "Task description"
  )
  public String task = "s.task.locomotion(terrain = s.t.flat())";
  @Parameter(
      names = {"--nOfThreads"},
      description = "Number of threads"
  )
  public int nOfThreads = 1;
  @Parameter(
      names = {"--randomSeed"},
      description = "Seed of the random generator"
  )
  public int randomSeed = 1;

  private record AnnotatedGenotype(
      String target,
      int seed,
      int iteration,
      List<Double> sourceGenotype,
      double q
  ) {}

  private record Outcome(
      AnnotatedGenotype annotatedSourceGenotype,
      int destinationIndex,
      int stepIndex,
      double d,
      List<Double> genotype,
      double q
  ) {}

  private static List<Double> dPoint(double d, List<Double> src, List<Double> direction) {
    if (src.size() != direction.size()) {
      throw new IllegalArgumentException("Vectors should have the same length: got %d and %d".formatted(
          src.size(),
          direction.size()
      ));
    }
    return IntStream.range(0, src.size())
        .mapToObj(i -> src.get(i) + direction.get(i) * d)
        .toList();
  }

  private static double distance(List<Double> p1, List<Double> p2) {
    if (p1.size() != p2.size()) {
      throw new IllegalArgumentException("Vectors should have the same length: got %d and %d".formatted(
          p1.size(),
          p2.size()
      ));
    }
    double d = 0;
    for (int i = 0; i < p1.size(); i++) {
      d = d + (p1.get(i) - p2.get(i)) * (p1.get(i) - p2.get(i));
    }
    return Math.sqrt(d);
  }

  public static List<Double> deserialize(String s) {
    try (
        ByteArrayInputStream bais = new ByteArrayInputStream(Base64.getDecoder().decode(s));
        ObjectInputStream ois = new ObjectInputStream(bais)
    ) {
      //noinspection unchecked
      return (List<Double>) ois.readObject();
    } catch (IOException | ClassNotFoundException e) {
      L.severe("Cannot deserialize: %s".formatted(e));
    }
    return null;
  }

  public static String serialize(List<Double> values) {
    try (ByteArrayOutputStream baos = new ByteArrayOutputStream();
         ObjectOutputStream oos = new ObjectOutputStream(baos)
    ) {
      oos.writeObject(values);
      return Base64.getEncoder().encodeToString(baos.toByteArray());
    } catch (IOException e) {
      L.severe("Cannot deserialize: %s".formatted(e));
    }
    return "";
  }

  public static void main(String[] args) {
    try {
      Starter starter = new Starter();
      JCommander.newBuilder()
          .addObject(starter)
          .build()
          .parse(args);
      starter.run();
    } catch (ParameterException e) {
      e.usage();
      L.severe(String.format("Cannot read command line options: %s", e));
      System.exit(-1);
    } catch (RuntimeException e) {
      L.severe(e.getClass().getSimpleName() + ": " + e.getMessage());
      System.exit(-1);
    }
  }

  private static List<Double> randomUnitVector(int p, RandomGenerator randomGenerator) {
    List<Double> v = IntStream.range(0, p).mapToObj(i -> randomGenerator.nextGaussian(0d, 1d)).toList();
    List<Double> origin = Collections.nCopies(p, 0d);
    double norm = distance(v, origin);
    return v.stream().map(c -> c / norm).toList();
  }

  @Override
  public void run() {
    NamedBuilder<?> nb = PreparedNamedBuilder.get();
    //read file and save bests
    List<AnnotatedGenotype> annotatedGenotypes;
    try (Reader reader = new FileReader(inputFile)) {
      L.info("Reading input file %s".formatted(inputFile));
      CSVParser parser = CSVFormat.Builder.create().setDelimiter(';').setHeader().build().parse(reader);
      annotatedGenotypes = parser.stream()
          .map(r -> {
            String target;
            if (parser.getHeaderNames().contains("solver.mapper.target")) {
              target = r.get("solver.mapper.target");
            } else {
              target = VSR_TARGET_TEMPLATE
                  .replace("%SHAPE%", r.get("solver.mapper.target.body.shape"))
                  .replace("%FUNCTION%", r.get("solver.mapper.target.function"))
                  .replaceFirst("free\\(s=([^)]*)\\)", "free(s=\"$1\")")
                  .replaceAll("\\s", "");
            }
            return new AnnotatedGenotype(
                target,
                Integer.parseInt(r.get("randomGenerator.seed")),
                Integer.parseInt(r.get("iterations")),
                deserialize(r.get("best→genotype→base64")),
                Double.parseDouble(r.get("best→fitness→s.task.l.xVelocity"))
            );
          })
          .toList();
      L.info("%d target genotypes found for iterations %s".formatted(annotatedGenotypes.size(), iterations));
    } catch (IOException e) {
      throw new IllegalArgumentException("Cannot open input file %s: %s".formatted(inputFile, e));
    }
    //show info and domains by target
    annotatedGenotypes.stream()
        .map(AnnotatedGenotype::target)
        .distinct()
        .forEach(t -> {
          List<Integer> gSizes = annotatedGenotypes.stream()
              .filter(ag -> ag.target().equals(t))
              .map(ag -> ag.sourceGenotype().size())
              .distinct()
              .toList();
          List<DoubleRange> genotypeRanges = annotatedGenotypes.stream()
              .filter(ag -> ag.target().equals(t))
              .map(ag -> ag.sourceGenotype.stream().map(v -> new DoubleRange(v, v)).toList())
              .reduce((rs1, rs2) -> IntStream.range(0, rs1.size()).mapToObj(i -> new DoubleRange(
                  Math.min(rs1.get(i).min(), rs2.get(i).min()),
                  Math.max(rs1.get(i).max(), rs2.get(i).max())
              )).toList())
              .orElseThrow();
          DoubleRange enclosingRange = genotypeRanges.stream().reduce((r1, r2) -> new DoubleRange(
              Math.min(r1.min(), r2.min()),
              Math.max(r1.max(), r2.max())
          )).orElseThrow();
          DoubleRange averageRange = genotypeRanges.stream().reduce((r1, r2) -> new DoubleRange(
              (r1.min() + r2.min()) / 2d,
              (r1.max() + r2.max()) / 2d
          )).orElseThrow();
          System.out.printf(
              """
                  Target: %s
                  Distinct genotype sizes: %s
                  Enclosing genes range: [%+.3f,%+.3f]
                  Average genes range: [%+.3f,%+.3f]
                  """,
              t,
              gSizes,
              enclosingRange.min(), enclosingRange.max(),
              averageRange.min(), averageRange.max()
          );
        });
    //prepare engine, executor, cFunction, task
    Supplier<Engine> engineSupplier = () -> ServiceLoader.load(Engine.class)
        .findFirst()
        .orElseThrow(() -> new RuntimeException("Cannot instantiate an engine"));
    ExecutorService executorService = Executors.newFixedThreadPool(nOfThreads);
    @SuppressWarnings("unchecked") Function<Object, Double> cFunction = (Function<Object, Double>) nb.build(C_FUNCTION);
    @SuppressWarnings("unchecked") Task<Supplier<EmbodiedAgent>, ?> localTask =
        (Task<Supplier<EmbodiedAgent>, ?>) nb.build(
            task);
    RandomGenerator randomGenerator = new Random(randomSeed);
    //iterate over bests
    List<Future<Outcome>> futures = new ArrayList<>();
    for (AnnotatedGenotype annotatedGenotype : annotatedGenotypes.stream()
        .filter(ag -> iterations.contains(ag.iteration))
        .toList()) {
      for (int destinationIndex = 0; destinationIndex < nOfDestinations; destinationIndex = destinationIndex + 1) {
        int dI = destinationIndex;
        List<Double> unitDiff = randomUnitVector(annotatedGenotype.sourceGenotype().size(), randomGenerator);
        for (int stepIndex = 0; stepIndex <= nOfSteps; stepIndex = stepIndex + 1) {
          int sI = stepIndex;
          double d = destinationDistance * (double) stepIndex / (double) nOfSteps;
          List<Double> genotype = dPoint(d, annotatedGenotype.sourceGenotype(), unitDiff);
          @SuppressWarnings("unchecked") InvertibleMapper<List<Double>, Supplier<EmbodiedAgent>> mapper =
              (InvertibleMapper<List<Double>, Supplier<EmbodiedAgent>>) nb.build(
                  MAPPER_TEMPLATE.replace("%TARGET%", annotatedGenotype.target()));
          Supplier<EmbodiedAgent> agentSupplier = mapper.apply(genotype);
          futures.add(executorService.submit(() -> {
            Object taskOutcome = localTask.run(agentSupplier, engineSupplier.get());
            return new Outcome(annotatedGenotype, dI, sI, d, genotype, cFunction.apply(taskOutcome));
          }));
        }
      }
    }
    L.info("%d tasks scheduled".formatted(futures.size()));
    //save results
    CSVPrinter printer;
    try {
      printer = new org.apache.commons.csv.CSVPrinter(
          new PrintStream(outputFile),
          CSVFormat.Builder.create().setDelimiter(";").build()
      );
      printer.printRecord(List.of(
          "target",
          "seed",
          "iteration",
          "srcQ",
          "dstIndex",
          "stepIndex",
          "d",
          "pointQ",
          "genotype"
      ));
    } catch (IOException e) {
      L.severe("Cannot open output file: %s".formatted(e));
      executorService.shutdownNow();
      return;
    }
    AtomicInteger counter = new AtomicInteger(0);
    futures.forEach(f -> {
      try {
        Outcome outcome = f.get();
        L.info("Outcome %d/%d for iteration %d, target %d, point %d found: %6.3f vs. %6.3f ".formatted(
            counter.incrementAndGet(),
            futures.size(),
            outcome.annotatedSourceGenotype().iteration(),
            outcome.destinationIndex(),
            outcome.stepIndex(),
            outcome.q(),
            outcome.annotatedSourceGenotype().q()
        ));
        printer.printRecord(List.of(
            outcome.annotatedSourceGenotype().target(),
            outcome.annotatedSourceGenotype().seed(),
            outcome.annotatedSourceGenotype().iteration(),
            outcome.annotatedSourceGenotype().q(),
            outcome.destinationIndex(),
            outcome.stepIndex(),
            outcome.d(),
            outcome.q(),
            serialize(outcome.genotype())
        ));
      } catch (InterruptedException | ExecutionException e) {
        L.severe("Cannot get result due to: %s".formatted(e));
      } catch (IOException e) {
        L.severe("Cannot print result due to: %s".formatted(e));
      }
    });

  }

}
