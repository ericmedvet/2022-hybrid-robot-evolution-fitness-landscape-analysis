/*
 * Copyright 2022 eric
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

package it.units.erallab.respap.hrefla;

import com.beust.jcommander.JCommander;
import com.beust.jcommander.Parameter;
import com.beust.jcommander.ParameterException;
import it.units.erallab.mrsim2d.builder.NamedBuilder;
import it.units.erallab.mrsim2d.core.Agent;
import it.units.erallab.mrsim2d.core.EmbodiedAgent;
import it.units.erallab.mrsim2d.core.engine.Engine;
import it.units.erallab.mrsim2d.core.tasks.Task;
import it.units.erallab.robotevo2d.main.builder.MapperBuilder;
import it.units.erallab.robotevo2d.main.builder.SerializerBuilder;
import it.units.erallab.robotevo2d.main.singleagent.PreparedNamedBuilder;
import it.units.malelab.jgea.core.listener.NamedFunction;
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
  public String task = "";
  @Parameter(
      names = {"--qExtractor"},
      description = "Quality extraction function"
  )
  public String qExtractor = "";
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
      String mapper,
      String randomGenerator,
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

  public static Function<String, Object> javaDeserializer() {
    return s -> {
      try (ByteArrayInputStream bais = new ByteArrayInputStream(Base64.getDecoder()
          .decode(s)); ObjectInputStream ois = new ObjectInputStream(
          bais)) {
        return ois.readObject();
      } catch (IOException | ClassNotFoundException e) {
        L.severe("Cannot deserialize: %s".formatted(e));
      }
      return null;
    };
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
      //noinspection unchecked
      annotatedGenotypes = parser.stream()
          .filter(r -> iterations.contains(Integer.parseInt(r.get("iterations"))))
          .map(r -> new AnnotatedGenotype(
              r.get("target"),
              r.get("mapper"),
              r.get("randomGenerator"),
              Integer.parseInt(r.get("iterations")),
              (List<Double>) javaDeserializer().apply(r.get("best→g")),
              Double.parseDouble(r.get("best→fitness→vx"))
          ))
          .toList();
      L.info("%d target genotypes found for iterations %s".formatted(annotatedGenotypes.size(), iterations));
    } catch (IOException e) {
      throw new IllegalArgumentException("Cannot open input file %s: %s".formatted(inputFile, e));
    }
    //prepare engine and executor and qExtractor
    Supplier<Engine> engineSupplier = () -> ServiceLoader.load(Engine.class)
        .findFirst()
        .orElseThrow(() -> new RuntimeException("Cannot instantiate an engine"));
    ExecutorService executorService = Executors.newFixedThreadPool(nOfThreads);
    @SuppressWarnings("unchecked") NamedFunction<Object, Double> qExtractorF =
        (NamedFunction<Object, Double>) nb.build(qExtractor);
    RandomGenerator randomGenerator = new Random(randomSeed);
    Function<Object, String> serializer = SerializerBuilder.javaSerializer();
    //iterate over bests
    List<Future<Outcome>> futures = new ArrayList<>();
    for (AnnotatedGenotype annotatedGenotype : annotatedGenotypes) {
      for (int destinationIndex = 0; destinationIndex < nOfDestinations; destinationIndex = destinationIndex + 1) {
        int dI = destinationIndex;
        List<Double> unitDiff = randomUnitVector(annotatedGenotype.sourceGenotype().size(), randomGenerator);
        for (int stepIndex = 0; stepIndex <= nOfSteps; stepIndex = stepIndex + 1) {
          int sI = stepIndex;
          double d = destinationDistance * (double) stepIndex / (double) nOfSteps;
          List<Double> genotype = dPoint(d, annotatedGenotype.sourceGenotype(), unitDiff);
          Supplier<EmbodiedAgent> unmappedAgentSupplier = () -> (EmbodiedAgent) nb.build(annotatedGenotype.target());
          @SuppressWarnings("unchecked") MapperBuilder<List<Double>, Supplier<EmbodiedAgent>> mapper =
              (MapperBuilder<List<Double>, Supplier<EmbodiedAgent>>) nb.build(
                  annotatedGenotype.mapper());
          Supplier<EmbodiedAgent> agentSupplier = mapper.buildFor(unmappedAgentSupplier).apply(genotype);
          @SuppressWarnings("unchecked") Task<Supplier<Agent>, ?> localTask =
              (Task<Supplier<Agent>, ?>) nb.build(task);
          futures.add(executorService.submit(() -> {
            Object taskOutcome = localTask.run(agentSupplier::get, engineSupplier.get());
            return new Outcome(annotatedGenotype, dI, sI, d, genotype, qExtractorF.apply(taskOutcome));
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
          "mapper",
          "randomGenerator",
          "iteration",
          "srcGenotype",
          "srcQ",
          "genotype",
          "dstIndex",
          "stepIndex",
          "d",
          "pointQ"
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
        L.info(("Outcome %d/%d for iteration %d, target %d, point %d found: " + qExtractorF.getFormat() + " vs. " + qExtractorF.getFormat()).formatted(
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
            outcome.annotatedSourceGenotype().mapper(),
            outcome.annotatedSourceGenotype().randomGenerator(),
            outcome.annotatedSourceGenotype().iteration(),
            serializer.apply(outcome.annotatedSourceGenotype().sourceGenotype()),
            outcome.annotatedSourceGenotype().q(),
            serializer.apply(outcome.genotype()),
            outcome.destinationIndex(),
            outcome.stepIndex(),
            outcome.d(),
            outcome.q()
        ));
      } catch (InterruptedException | ExecutionException e) {
        L.severe("Cannot get result due to: %s".formatted(e));
      } catch (IOException e) {
        L.severe("Cannot print result due to: %s".formatted(e));
      }
    });
  }

}
