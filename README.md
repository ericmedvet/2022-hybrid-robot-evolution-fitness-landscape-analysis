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

Here you perform a number of evolutionary runs with 3 robotic agents: a legged robot with an open-loop controller and two voxel-based soft robots (VSRs), one with an open-loop controller, one with a closed-loop controller.
Moreover, for each agent, you consider a few (5 for the VSRs, 6 for the legged robots) variants that differ in the amount of soft components its body contains.
The two experiments descriptions are in [`phase-1/walker-3chunks-6x.txt`](phase-1/walker-3chunks-6x.txt) and [`phase-1/hybrid-vsr.txt`](phase-1/hybrid-vsr.txt).
The commands for starting the experiments are (assuming you are in the root project directory):
```shell
cd phase-1
java -cp "../io.github.ericmedvet.respap.hrefla.assembly/target/respap-hrefla.assembly-0.0.2-bin/modules/*" io.github.ericmedvet.robotevo2d.main.Starter --expFile best-walker-3chunks-6x.txt --nOfThreads 70
```
and, for the VSRs:
```shell
cd phase-1
java -cp "../io.github.ericmedvet.respap.hrefla.assembly/target/respap-hrefla.assembly-0.0.2-bin/modules/*" io.github.ericmedvet.robotevo2d.main.Starter --expFile hybrid-vsr.txt --nOfThreads 70
```
The param `70` is for the number of concurrent threads to be used.
It is reasonable to set a number that is equal to the number of cores on your machine.

Running each one of these experiments on our machine (Intel(R) Xeon(R) W-2295 CPU @ 3.00GHz with 64GB RAM) took approx 72h.

The outcome of the experiments are the CSV files named [`phase-1/best-walker-3chunks-6x.csv`](phase-1/best-walker-3chunks-6x.csv) and [`phase-1/best-hybrid-vsr.csv`](phase-1/best-hybrid-vsr.csv).

### Phase 2: search space walks

Here you perform the analysis of the fitness landscape.
In brief, for each run and each iteration number, you take the best individual at that iteration and build a number (`-nd` below) of *sections* of fitness landscape.
For each section, a number (`-np` below) of genotypes are built by evenly sampling the segment connecting the genotype of the best individual and a random genotype at a Euclidean distance (`-d` below) from the best individual genotype.
For each genotype (point) on the segment, you build a robot and compute its fitness on the task (`--task` below, the same of phase 1).

You repeat the process for the two files produced in phase 1.
The outcome of the experiments are two CSV files named [`legged-sections.csv`](phase-2/legged-sections.csv) and [`vsr-sections.csv`](phase-2/vsr-sections.csv).

For the legged robots:
```shell
cd phase-2
java -cp "../io.github.ericmedvet.respap.hrefla.assembly/target/respap-hrefla.assembly-0.0.2-bin/modules/*" io.github.ericmedvet.respap.hrefla.Starter -if ../phase-1/best-walker-3chunks-6x.csv -of legged-sections.csv -d 1.0 -i 0,49,99 -nd 5 -np 100 --nOfThreads 70 --task 's.task.locomotion(terrain=s.t.steppy(chunkW=2;chunkH=0.25);duration=30)'
```
and for the VSRs:
```shell
cd phase-2
java -cp "../io.github.ericmedvet.respap.hrefla.assembly/target/respap-hrefla.assembly-0.0.2-bin/modules/*" io.github.ericmedvet.respap.hrefla.Starter -if ../phase-1/best-hybrid-vsr.csv -of vsr-sections.csv -d 1.0 -i 0,49,99 -nd 5 -np 100 --nOfThreads 70 --task 's.task.locomotion(terrain=s.t.flat())'
```

### Phase 3: data analysis

For the post-processing of the experimental data, including the generation of the figure, use the R notebook [`phase-3/analysis.Rmd`](phase-3/analysis.Rmd).
