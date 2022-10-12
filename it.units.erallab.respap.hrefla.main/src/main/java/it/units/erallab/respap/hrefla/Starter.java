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

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.List;
import java.util.logging.Logger;

public class Starter implements Runnable {

  private final static Logger L = Logger.getLogger(Starter.class.getName());

  @Parameter(
      names = {"--evoResultFile", "-f"},
      description = "Path of the file with the results of the evolution"
  )
  public String evoResultsFile = "";
  @Parameter(
      names = {"--iteration", "-i"},
      description = "Iteration index to extract best individuals"
  )
  public List<Integer> iterations = List.of();
  @Parameter(
      names = {"--nOfTargets", "-nt"},
      description = "Number of target genotypes for each best"
  )
  public int nOfTargets = 1;
  @Parameter(
      names = {"--nOfPoints", "-np"},
      description = "Number of points for each best-to-target section"
  )
  public int nOfPoints = 10;

  private record TargetGenotype(
      String target
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

  @Override
  public void run() {
    //check data
    try (BufferedReader br = new BufferedReader(new FileReader(evoResultsFile))) {

    } catch (IOException e) {
      throw new IllegalArgumentException("Cannot open file %s: %s".formatted(evoResultsFile, e));
    }
    //read file and save bests
    //iterate over bests
  }

}
