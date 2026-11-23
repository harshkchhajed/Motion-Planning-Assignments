# RBE 550 Motion Planning: Assignments

Coursework for RBE 550 Motion Planning at WPI, written in C++ with [OMPL](https://ompl.kavrakilab.org/) and run in Docker. Each project folder has the code and a written report (PDF). Done with partner Pranay Katyal (Project 0 individually).

| Project | Topic |
|---|---|
| `Project0` | OMPL setup in Docker and running/modifying the `Point2DPlanning` demo |
| `Project1` | Uniform sampling of the unit disk: a naive versus a correct sampler, with plots, plus a disk-planning problem |
| `Project2` | Bug 1 versus Bug 2 analysis, then collision checking and planner benchmarking in OMPL (graph states, solution length, time) |
| `Project3` | Visibility graph versus PRM, with benchmark runs and planned-path visualizations |
| `Project4` | Kinodynamic planning (car and pendulum) with a custom Reachability-Guided RRT (RG-RRT) |

The final project, Chance-Constrained RRT, is in [MotionPlanningProjects](https://github.com/harshkchhajed/MotionPlanningProjects/tree/main/FinalProject/code).

## Build and run

Each project builds with CMake inside the OMPL Docker image. The exact Docker commands are in the `Docker commands.txt` file in each folder.

```bash
cd ProjectN/code
mkdir -p build && cd build
cmake .. && make
```

## Stack

C++, OMPL, CMake, Docker, Python (visualization).
