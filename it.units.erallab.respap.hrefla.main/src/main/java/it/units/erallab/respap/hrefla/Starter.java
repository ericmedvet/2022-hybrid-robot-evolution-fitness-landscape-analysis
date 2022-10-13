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
import it.units.erallab.robotevo2d.main.singleagent.PreparedNamedBuilder;
import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVParser;

import java.io.*;
import java.util.ArrayList;
import java.util.Base64;
import java.util.List;
import java.util.ServiceLoader;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.logging.Logger;

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
      names = {"--nOfTargets", "-nt"},
      description = "Number of target genotypes for each best"
  )
  public int nOfTargets = 1;
  @Parameter(
      names = {"--targetDistance", "-d"},
      description = "Euclidean distance of the target"
  )
  public double targetDistance = 0.5;
  @Parameter(
      names = {"--nOfPoints", "-np"},
      description = "Number of points for each best-to-target section"
  )
  public int nOfPoints = 10;
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

  private record Outcome(
      TargetGenotype targetGenotype,
      int targetIndex,
      int pointIndex,
      double d,
      List<Double> genotype,
      double q
  ) {}

  private record TargetGenotype(
      String target,
      String mapper,
      String randomGenerator,
      int iteration,
      List<Double> genotype
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

  public static Function<String, Object> javaDeserializer() {
    return s -> {
      try (ByteArrayInputStream bais = new ByteArrayInputStream(Base64.getDecoder().decode(s)); ObjectInputStream ois = new ObjectInputStream(
          bais)) {
        return ois.readObject();
      } catch (IOException | ClassNotFoundException e) {
        L.severe("Cannot deserialize: %s".formatted(e));
      }
      return null;
    };
  }

  @Override
  public void run() {
    NamedBuilder<?> nb = PreparedNamedBuilder.get();
    //read file and save bests
    List<TargetGenotype> targetGenotypes;
    try (Reader reader = new FileReader(inputFile)) {
      L.info("Reading input file %s".formatted(inputFile));
      CSVParser parser = CSVFormat.Builder.create().setDelimiter(';').setHeader().build().parse(reader);
      //noinspection unchecked
      targetGenotypes = parser.stream()
          .filter(r -> iterations.contains(Integer.parseInt(r.get("iterations"))))
          .map(r -> new TargetGenotype(
              r.get("target"),
              r.get("mapper"),
              r.get("randomGenerator"),
              Integer.parseInt(r.get("iterations")),
              (List<Double>)javaDeserializer().apply(r.get("best→g"))
          ))
          .toList();
      L.info("%d target genotypes found for iterations %s".formatted(targetGenotypes.size(), iterations));
    } catch (IOException e) {
      throw new IllegalArgumentException("Cannot open file %s: %s".formatted(inputFile, e));
    }
    //prepare engine and executor and qExtractor
    Supplier<Engine> engineSupplier = () -> ServiceLoader.load(Engine.class)
        .findFirst()
        .orElseThrow(() -> new RuntimeException("Cannot instantiate an engine"));
    ExecutorService executorService = Executors.newFixedThreadPool(nOfThreads);
    @SuppressWarnings("unchecked") Function<Object, Double> qExtractorF =
        (Function<Object, Double>) nb.build(qExtractor);
    //iterate over bests
    List<Future<Outcome>> futures = new ArrayList<>();
    for (TargetGenotype targetGenotype : targetGenotypes) {
      for (int targetIndex = 0; targetIndex < nOfTargets; targetIndex = targetIndex + 1) {
        int t = targetIndex;
        //TODO build random target point
        for (int pointIndex = 0; pointIndex < nOfPoints; pointIndex = pointIndex + 1) {
          int p = pointIndex;
          double d = 0; //TODO
          List<Double> genotype = targetGenotype.genotype(); //TODO
          Supplier<EmbodiedAgent> unmappedAgentSupplier = () -> (EmbodiedAgent) nb.build(targetGenotype.target());
          @SuppressWarnings("unchecked") MapperBuilder<List<Double>, Supplier<EmbodiedAgent>> mapper =
              (MapperBuilder<List<Double>, Supplier<EmbodiedAgent>>) nb.build(
                  targetGenotype.mapper());
          Supplier<EmbodiedAgent> agentSupplier = mapper.buildFor(unmappedAgentSupplier).apply(genotype);
          @SuppressWarnings("unchecked") Task<Supplier<Agent>, ?> localTask =
              (Task<Supplier<Agent>, ?>) nb.build(task);
          futures.add(executorService.submit(() -> {
            Object taskOutcome = localTask.run(agentSupplier::get, engineSupplier.get());
            L.info("Task done with outcome %s".formatted(taskOutcome));
            return new Outcome(targetGenotype, t, p, 0, genotype, qExtractorF.apply(taskOutcome));
          }));
        }
      }
    }
    //save results
    futures.forEach(f -> {
      try {
        L.info("%s".formatted(f.get())); // TODO save
      } catch (InterruptedException | ExecutionException e) {
        L.severe("Cannot get results due to: %s".formatted(e));
      }
    });
  }

}
