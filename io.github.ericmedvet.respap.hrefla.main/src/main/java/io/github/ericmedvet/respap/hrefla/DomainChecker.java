package io.github.ericmedvet.respap.hrefla;

import io.github.ericmedvet.jgea.core.IndependentFactory;
import io.github.ericmedvet.jgea.core.representation.sequence.FixedLengthListFactory;
import io.github.ericmedvet.jgea.experimenter.InvertibleMapper;
import io.github.ericmedvet.jnb.core.NamedBuilder;
import io.github.ericmedvet.mrsim2d.core.EmbodiedAgent;
import io.github.ericmedvet.mrsim2d.core.engine.Engine;
import io.github.ericmedvet.mrsim2d.core.tasks.Task;
import io.github.ericmedvet.robotevo2d.main.PreparedNamedBuilder;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.ServiceLoader;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.random.RandomGenerator;

/**
 * @author "Eric Medvet" on 2023/01/18 for 2022-hybrid-robot-evolution-fitness-landscape-analysis
 */
public class DomainChecker {

  private final static String VSR_OPEN_MAPPER = """
      s.a.centralizedNumGridVSR(
        body = s.a.vsr.gridBody(
          sensorizingFunction = s.a.vsr.sf.directional(
            headSensors = [s.s.sin(f = 0); s.s.d(a = -30; r = 10)];
            nSensors = [s.s.ar(); s.s.rv(a = 0); s.s.rv(a = 90)];
            sSensors = [s.s.d(a = -90)]
          );
          shape = s.a.vsr.s.free(s = "rsssr-sssss-s...s")
        );
        function = s.f.sin(
          p = s.range(min = -1.57; max = 1.57);
          f = s.range(min = 0.3; max = 2);
          a = s.range(min = 1; max = 1);
          b = s.range(min = 1; max = 1)
        )
      )
      """;

  private final static String VSR_CLOSED_MAPPER = """
      s.a.centralizedNumGridVSR(
        body = s.a.vsr.gridBody(
          sensorizingFunction = s.a.vsr.sf.directional(
            headSensors = [s.s.sin(f = 0); s.s.d(a = -30; r = 10)];
            nSensors = [s.s.ar(); s.s.rv(a = 0); s.s.rv(a = 90)];
            sSensors = [s.s.d(a = -90)]
          );
          shape = s.a.vsr.s.free(s = "rsssr-sssss-s...s")
        );
        function = s.f.mlp()
      )
      """;

  private final static String LEGGED_OPEN_MAPPER = """
      s.a.numLeggedHybridRobot(
        trunkLength = 35;
        trunkMass = 1;
        legs = [
          s.a.l.leg(downConnector = soft; legChunks = 3 * [s.a.l.legChunk()]);
          s.a.l.leg(downConnector = soft; legChunks = 3 * [s.a.l.legChunk()]);
          s.a.l.leg(downConnector = soft; legChunks = 3 * [s.a.l.legChunk()]);
          s.a.l.leg(downConnector = soft; legChunks = 3 * [s.a.l.legChunk()]);
          s.a.l.leg(downConnector = soft; legChunks = 3 * [s.a.l.legChunk()])
        ];
        function = s.f.groupedSin(
          size = 3;
          p = s.range(min = -1.57; max = 1.57);
          f = s.range(min = 0.3; max = 2);
          a = s.range(min = 0.0; max = 0.5);
          b = s.range(min = -1; max = 1);
          s = s.range(min = -0.5; max = 0.5)
        )
      )
      """;

  private final static String LEGGED_OPEN_2_MAPPER = """
      s.a.numLeggedHybridRobot(
        trunkLength = 10;
        trunkMass = 1;
        legs = [
          s.a.l.leg(downConnector = soft; legChunks = 3 * [s.a.l.legChunk()]);
          s.a.l.leg(downConnector = soft; legChunks = 3 * [s.a.l.legChunk()])
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

  private final static String TASK = "s.task.locomotion(terrain = s.t.flat())";
  private final static String C_FUNCTION = "s.task.locomotion.xVelocity()";
  private final static String MAPPER_TEMPLATE = "er.m.parametrizedHomoBrains(target=%TARGET%)";

  public static void main(String[] args) {
    NamedBuilder<?> nb = PreparedNamedBuilder.get();
    Supplier<Engine> engineSupplier = () -> ServiceLoader.load(Engine.class)
        .findFirst()
        .orElseThrow(() -> new RuntimeException("Cannot instantiate an engine"));
    @SuppressWarnings("unchecked") Function<Object, Double> cFunction = (Function<Object, Double>) nb.build(C_FUNCTION);
    @SuppressWarnings("unchecked") Task<Supplier<EmbodiedAgent>, ?> task =
        (Task<Supplier<EmbodiedAgent>, ?>) nb.build(
            TASK);
    RandomGenerator randomGenerator = new Random(1);
    @SuppressWarnings("unchecked") InvertibleMapper<List<Double>, Supplier<EmbodiedAgent>> mapper =
        (InvertibleMapper<List<Double>, Supplier<EmbodiedAgent>>) nb.build(
            MAPPER_TEMPLATE.replace("%TARGET%", LEGGED_OPEN_2_MAPPER));
    IndependentFactory<List<Double>> factory = new FixedLengthListFactory<>(
        mapper.exampleInput().size(),
        r -> r.nextDouble(-1, 1)
    );
    System.out.println(factory.build(randomGenerator).size());
    List<Double> g1 = factory.build(randomGenerator);
    g1.set(1, 1d);
    List<Double> g2 = new ArrayList<>(g1);
    g1.set(1, 1.1d);
    System.out.println(cFunction.apply(task.run(mapper.apply(g1), engineSupplier.get())));
    System.out.println(cFunction.apply(task.run(mapper.apply(g1), engineSupplier.get())));
    System.out.println(cFunction.apply(task.run(mapper.apply(g2), engineSupplier.get())));
    System.out.println(cFunction.apply(task.run(mapper.apply(g2), engineSupplier.get())));
  }
}
