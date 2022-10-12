# Experimental tools and artifacts for a research on: Fitness landscape analysis of hyrbid (soft/rigid) robot evolution

This repo includes the tools and the artifacts needed for replicating the results of an experimental research on *the impact of softness on the ruggedness of the fitness landscape in the evolutionary optimization of hybrid (soft/rigid) legged robots*.

## Dependencies

This project depends on:
- [2d-robot-evolution](https://github.com/ericmedvet/2d-robot-evolution) v0.3.2, which in turn depends on:
  - [JGEA](https://github.com/ericmedvet/jgea) v2.2.1
  - [2-D Multi Robot Simulator (2D-MR-Sim)](https://github.com/ericmedvet/2dmrsim)
- Java SDK 18
- R (with some appropriate tools)

## Overview of the experimental phases

### Preparation

After having cloned this repo, build it with:
```bash
mvn clean install
```

Note that the `2d-robot-evolution` Maven dependency is not on the central repository.
Before building this project, you have to clone `2d-robot-evolution` (the proper release) on your machine and build it with `mvn clean install`.

### Phase 1: evolution

Here you perform a number of evolutionary runs with 3 different bodies (no soft parts, 5 soft parts, 10 soft parts) and 4 different open-loop controllers (combinations of phase, amplitude, and frequency of a periodic controller).
The experiment description is in [`phase-1/exp-legged.txt`](phase-1/exp-legged.txt).
The command for starting the experiment is:
```bash
cd phase-1
java -cp "../it.units.erallab.respap.hrefla.assembly/target/respap-hrefla.assembly-0.0.1-bin/modules/*" it.units.erallab.robotevo2d.main.singleagent.Starter --expFile exp-legged.txt --nOfThreads 70
```
The param `70` is for the number of concurrent threads to be used.
It is reasonable to set a number that is equal to the number of cores on your machine.

Running this experiments on our machine (Intel(R) Xeon(R) W-2295 CPU @ 3.00GHz with 64GB RAM) took approx 72h.

The outcome of the experiment is the CSV file named [`results.txt`](phase-1/results.txt).

### Phase 2: search space walks

### Phase 3: data analysis

For the post-processing of the experimental data, including the generation of the figure, use the R notebook [`phase-3/analysis.Rmd`](phase-3/analysis.Rmd).
