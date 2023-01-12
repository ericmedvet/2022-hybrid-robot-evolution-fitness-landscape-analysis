# Experimental tools and artifacts for a research on: Fitness landscape analysis of hyrbid (soft/rigid) robot evolution

This repo includes the tools and the artifacts needed for replicating the results of an experimental research on *the impact of softness on the ruggedness of the fitness landscape in the evolutionary optimization of hybrid (soft/rigid) simulated robots*.

## Dependencies

This project depends on:
- [2d-robot-evolution](https://github.com/ericmedvet/2d-robot-evolution), which in turn depends on:
- Java SDK 17
- R (with some appropriate tools and packages)

## Overview of the experimental phases

### Preparation

After having cloned this repo, build it with:
```shell
mvn clean package
```

### Phase 1: evolution

Here you perform a number of evolutionary runs with 3 different bodies (no soft parts, 5 soft parts, 10 soft parts) and 4 different open-loop controllers (combinations of phase, amplitude, and frequency of a periodic controller).
The experiment description is in [`phase-1/exp-legged.txt`](phase-1/exp-legged.txt).
The command for starting the experiment is:
```shell
cd phase-1
java -cp "../it.units.erallab.respap.hrefla.assembly/target/respap-hrefla.assembly-0.0.1-bin/modules/*" it.units.erallab.robotevo2d.main.singleagent.Starter --expFile exp-legged.txt --nOfThreads 70
```
The param `70` is for the number of concurrent threads to be used.
It is reasonable to set a number that is equal to the number of cores on your machine.

Running this experiments on our machine (Intel(R) Xeon(R) W-2295 CPU @ 3.00GHz with 64GB RAM) took approx 72h.

The outcome of the experiment is the CSV file named [`results-evo.txt`](phase-1/results-evo.txt).

### Phase 2: search space walks

Here you perform the analysis of the fitness landscape.
In brief, for each run and each iteration number, you take the best individual at that iteration and build a number (`-nd` below) of *sections* of fitness landscape.
For each section, a number (`-np` below) of genotypes are built by evenly sampling the segment connecting the genotype of the best individual and a random genotype at a Euclidean distance (`-d` below) from the best individual genotype.
For each genotype (point) on the segment, you build a robot and compute its fitness on the task (`--task` below, the same of phase-1).

The outcome of the experiment is the CSV file named [`sections.txt`](phase-2/sections.txt).

```shell
cd phase-2
java -cp "../it.units.erallab.respap.hrefla.assembly/target/respap-hrefla.assembly-0.0.1-bin/modules/*" it.units.erallab.respap.hrefla.Starter -if ../phase-1/results-evo.txt -of sections.txt -d 0.25 -i 1,100,199 -nd 10 -np 25 --nOfThreads 70 --qExtractor 'e.locomotionXVelocity()' --task 's.task.locomotion(terrain=s.t.flat();duration=45)'
```

### Phase 3: data analysis

For the post-processing of the experimental data, including the generation of the figure, use the R notebook [`phase-3/analysis.Rmd`](phase-3/analysis.Rmd).
