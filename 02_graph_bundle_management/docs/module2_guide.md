# Module 2 guide: graph and bundle management

This guide explains what each part of Module 2 does and how to run it without
ROS, Gazebo, or RViz. Module 2 is the stateful planning core between geometric
perception (Module 1) and navigation/motion planning (Module 3).

## 1. What Module 2 does

Each observation from Module 1 contains the robot pose, neighbor-sight
geometry, visible boundaries, and open points. Module 2 turns those observations
into persistent planning state:

1. open points are validated, merged, classified, and ranked;
2. observation centers and open points are accumulated in a visibility graph;
3. graph search produces a skeleton path to a selected open point;
4. the corresponding observation bundles are reconstructed in travel order;
5. gate preparation reports a validated result or an explicit typed failure for
   geometry that is not yet part of the Module 3 contract.

The code is deliberately ROS-independent. ROS nodes, simulation, and
visualization can be added around this API later without changing the core
algorithm.

## 2. Directory map

| Directory | Responsibility |
| --- | --- |
| `ranking_memory/` | Stores open-point records, lifecycle state, active-point filtering, and paper-based ranking. |
| `visibility_graph/` | Maintains observation-center/open-point nodes, evidence-backed edges, and BFS/Dijkstra search. |
| `bundle_sequences/` | Builds bundles from visible boundaries, stores bundle history, extracts route-ordered sequences, and validates gate-preparation inputs. |
| `pipeline/` | Coordinates the complete update and route-preparation flow through `GraphBundleManager`. |
| `include/` | Public C++17 headers consumed by Module 3 or offline callers. |
| `internal/` | Shared validation, geometry, and deterministic-ID helpers used only by M2 implementation files. |
| `examples/` | A small synthetic update that exercises the public manager without ROS. |
| `tests/` | Focused acceptance tests for ranking, graph operations, bundles, sequences, and the manager pipeline. |

## 3. Main data flow

The normal call sequence is:

```cpp
mars::graph_bundle_management::GraphBundleManager manager;

auto summary = manager.update(update);
auto ranked = manager.ranked_open_points();
auto route = manager.prepare_route(ranked.front().id);
```

`update` contains a stable observation ID, current pose, goal, and one Module 1
`PerceptionResult`. The manager validates the boundary contract first, then
updates memory, graph, and bundle history. `prepare_route` searches the
accumulated graph, resolves the bundles for the selected route, and returns the
handoff object for Module 3.

Important ownership boundaries:

- Module 1 owns perception geometry and `PerceptionResult`.
- Module 2 owns exploration memory, visibility connectivity, bundles, and route
  ordering.
- Module 3 owns DAP/funnel/motion planning.
- ROS, simulation, and visualization are outside this core library.

## 4. Build and run

From the repository root (`MARS_Modular`):

```sh
cmake -S . -B build -DMARS_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

The test target is registered with CTest. To run the standalone synthetic
example after building:

```sh
./build/02_graph_bundle_management/mars_graph_bundle_management_example
```

On Windows with a multi-config generator, use the configuration directory:

```powershell
.\build\02_graph_bundle_management\Debug\mars_graph_bundle_management_example.exe
```

The exact binary path can differ by generator; CMake's build output lists the
actual location. The example uses synthetic geometry and demonstrates the
public API; it does not open a simulator or display a visualization.

## 5. Failure behavior

Malformed geometry and invalid configuration raise `std::invalid_argument`.
Normal planning absence is represented as a typed failure: an unknown target,
disconnected graph, missing bundle, or unsupported gate geometry is not
silently replaced with a straight-line route or fabricated gate.

For the paper-to-code mapping and the intentionally unsupported legacy
heuristics, see the implementation notes in
[`IMPLEMENTATION_PLAN.md`](../IMPLEMENTATION_PLAN.md).
