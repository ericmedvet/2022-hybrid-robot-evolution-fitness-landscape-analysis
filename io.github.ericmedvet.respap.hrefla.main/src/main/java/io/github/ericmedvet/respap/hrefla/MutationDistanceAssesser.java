package io.github.ericmedvet.respap.hrefla;

import io.github.ericmedvet.jgea.core.IndependentFactory;
import io.github.ericmedvet.jgea.core.distance.Distance;
import io.github.ericmedvet.jgea.core.distance.LNorm;
import io.github.ericmedvet.jgea.core.operator.Mutation;
import io.github.ericmedvet.jgea.core.representation.sequence.FixedLengthListFactory;
import io.github.ericmedvet.jgea.core.representation.sequence.numeric.GaussianMutation;

import java.util.List;
import java.util.Locale;
import java.util.Random;
import java.util.random.RandomGenerator;
import java.util.stream.IntStream;

/**
 * @author "Eric Medvet" on 2023/01/18 for 2022-hybrid-robot-evolution-fitness-landscape-analysis
 */
public class MutationDistanceAssesser {

  public static void main(String[] args) {
    int n = 1000;
    List<Double> sigmas = List.of(0.05,0.1, 0.2, 0.3, 0.35,0.4,0.5);
    List<Integer> ps = List.of(10, 25, 50, 100, 400);
    Distance<double[]> lNorm2 = new LNorm(2d);
    Distance<List<Double>> distance = (v1, v2) -> lNorm2.apply(
        v1.stream().mapToDouble(v -> v).toArray(),
        v2.stream().mapToDouble(v -> v).toArray()
    );
    RandomGenerator rnd = new Random();
    for (int p : ps) {
      IndependentFactory<List<Double>> factory = new FixedLengthListFactory<>(p, r -> r.nextDouble(-1, 1));
      for (double sigma : sigmas) {
        Mutation<List<Double>> mutation = new GaussianMutation(sigma);
        System.out.printf(Locale.ROOT,"%3d %5.3f %6.3f%n",
            p, sigma,
            IntStream.range(0, n).mapToDouble(i -> {
              List<Double> g = factory.build(rnd);
              return distance.apply(g, mutation.mutate(g, rnd));
            }).average().orElseThrow()
        );
      }
    }
  }
}
