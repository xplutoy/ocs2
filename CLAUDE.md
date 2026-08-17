# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository context

OCS2 (**O**ptimal **C**ontrol for **S**witched **S**ystems) is a C++ toolbox for nonlinear optimal control and real-time MPC for robotics. This is a **ROS 2 port**: the active branch here is `ros2` (Ubuntu 24.04 + ROS 2 Jazzy, colcon/ament). The `main` branch is the original ROS 1/catkin version — build/launch commands and CI differ between branches, so confirm which branch you are on before following any instructions. The online docs at https://leggedrobotics.github.io/ocs2/ still describe the ROS 1 workflow; the solver concepts and C++ APIs are largely identical.

Two packages are **not ported to Jazzy** and are excluded via `COLCON_IGNORE`: `ocs2_mpcnet`, `ocs2_raisim`. `rqt_multiplot` is unavailable on Jazzy, so multiplot launch files are optional.

## Build & test

This is a ROS 2 colcon workspace, not a standalone CMake project. Build it as part of a workspace that also contains `ocs2_robotic_assets` (branch `ros2`) and a sparse-checkout of `elevation_mapping_cupy/plane_segmentation` (branch `ros2`) — several examples/tests reference assets from those repos. Full setup (deps, robotpkg Pinocchio) is in [`installation.md`](installation.md).

```bash
source /opt/ros/jazzy/setup.bash
# from workspace root (parent of src/), with this repo at src/ocs2
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

- Pinocchio comes from OpenRobots **robotpkg** (rosdep resolves it to an unreleased `ros-jazzy-pinocchio`), so always `--skip-keys pinocchio` with rosdep and export `CMAKE_PREFIX_PATH=/opt/openrobots:...`.
- `-GNinja` speeds up builds significantly (CI uses it). Pass it via `--cmake-args -GNinja`.
- Build a subset: `colcon build --packages-up-to <pkg>` (pkg + its deps) or `--packages-select <pkg>`.
- Docs are off by default: `colcon build --packages-select ocs2_doc --cmake-args -DBUILD_DOCS=ON` (HTML at `build/ocs2_doc/output/sphinx/index.html`).
- Optional Docker: `docker build -f docker/Dockerfile.jazzy -t ocs2:jazzy .` (clones the robotic assets + plane_segmentation itself).

### Tests

Tests use `ament_cmake_gtest` (declared per-target with `ament_add_gtest(...)` in each package's `CMakeLists.txt` under `BUILD_TESTING`). `BUILD_TESTING` is on by default in Release; CI's Debug matrix turns it `OFF`.

```bash
colcon test --event-handlers console_direct+          # all packages
colcon test --packages-select ocs2_core               # one package
colcon test --packages-select ocs2_core --ctest-args -R <gtest_target_name>   # one test target
colcon test-result --verbose                          # summary
```

To run a single gtest binary directly with gtest filtering: `build/<pkg>/<test_target> --gtest_filter=TestSuiteName.TestName`.

### Run an example

```bash
ros2 launch ocs2_ballbot_ros ballbot_mpc_mrt.launch.py   # in-process MRT
ros2 launch ocs2_ballbot_ros ballbot_sqp.launch.py       # ROS-interface variant per solver
```

## Architecture

The code is layered bottom-up; each layer is a package (or set of packages) that depends only on lower layers. All C++ lives in namespace `ocs2`. Canonical scalar/vector/matrix types are defined once in [ocs2_core/include/ocs2_core/Types.h](ocs2_core/include/ocs2_core/Types.h) (`scalar_t`, `vector_t`, `matrix_t`, plus `*_array_t` trajectory types and the `ScalarFunction*Approximation`/`VectorFunction*Approximation` structs used everywhere for Taylor expansions).

**ocs2_core** — math primitives and the data model. Subsystems: `dynamics` (`SystemDynamicsBase`), `cost` (`StateCost`/`StateInputCost` + Collections), `constraint` (`StateConstraint`/`StateInputConstraint` + Collections), `augmented_lagrangian`, `soft_constraint`, `penalties` (relaxed-barrier / squared-hinge), `integration` (RKDP5, sensitivity integrators, event handlers), `loopshaping` (transforms a problem to add dynamics/cost/constraint filters — *Eliminate*/*Output* patterns), `thread_support` (`ThreadPool`), `automatic_differentation` (`CppAdInterface`), and `misc/LoadData.h` providing `loadData::loadEigenMatrix` / `loadCppDataType` to read `.info` config files (Boost property-tree based). `ocs2_thirdparty` vendors CppAd headers.

**ocs2_oc** — the optimal-control layer that all solvers share. [OptimalControlProblem.h](ocs2_oc/include/ocs2_oc/oc_problem/OptimalControlProblem.h) is the central problem definition: a struct aggregating cost, soft-constraint, equality/inequality constraint, and Lagrangian collections, each split into **intermediate** (state-input), **state-only**, **pre-jump**, and **final** stages. [SolverBase.h](ocs2_oc/include/ocs2_oc/oc_solver/SolverBase.h) is the common solver interface (`run`, `reset`, `getPrimalSolution`, `getValueFunction`, `getHamiltonian`, ...). Shared machinery lives here: `rollout`, `multiple_shooting`, `search_strategy` (step-length/line-search), `precondition`, `trajectory_adjustment`, and `synchronized_module` (`ReferenceManager`, `SolverSynchronizedModule`, `SolverObserver` — updated pre/post solve).

**Solvers** — each implements `SolverBase`, each with its own `*Settings.h` loaded from `.info` files:
- `ocs2_ddp` — SLQ (continuous-time constrained DDP) and iLQR (discrete-time).
- `ocs2_sqp` — multiple-shooting SQP; QP subproblems via HPIPM/BLASFEO, built in the `blasfeo_catkin`/`hpipm_catkin` subpackages (these keep their `*_catkin` names but are ROS 2 `ament_cmake` packages on this branch).
- `ocs2_slp` — sequential linear programming (PIPG).
- `ocs2_ipm` — multiple-shooting nonlinear interior-point.
- `ocs2_frank_wolfe` — constrained Frank-Wolfe / NLP.

**ocs2_mpc** — wraps a solver in the MPC loop. `MPC_BASE` runs the solver and exposes `getSolverPtr`/`calculateController`. `MRT_BASE` + `MPC_MRT_Interface` drive MPC **in-process** (single-thread, shares the solver object). The **ROS** variants (decoupled MPC node + communication) live in `ocs2_ros_interfaces`: `MRT_ROS_Interface`, `MPC_ROS_Interface`, `MRT_ROS_Dummy_Loop`, plus command publishers (`TargetTrajectoriesRosPublisher`, keyboard/interactive-marker variants) and `common/RosMsgConversions.cpp` mapping between C++ types and `ocs2_msgs` messages.

**ocs2_python_interface** — [PythonInterface.h](ocs2_python_interface/include/ocs2_python_interface/PythonInterface.h) is the base class robot examples subclass; [PybindMacros.h](ocs2_python_interface/include/ocs2_python_interface/PybindMacros.h) defines `CREATE_ROBOT_PYTHON_BINDINGS(PY_INTERFACE, LIB_NAME)`, which generates a full `mpc_interface` python module (opaque `scalar_array`/`vector_array`/`matrix_array`, approximation structs, `TargetTrajectories`, and the MPC API). `LIB_NAME` **must match** the `pybind11_add_module` target name in CMake.

**Robotics tooling** — `ocs2_pinocchio` (`ocs2_pinocchio_interface`, `ocs2_centroidal_model`, `ocs2_self_collision` via HPP-FCL, `ocs2_sphere_approximation`), `ocs2_robotic_tools` (`RobotInterface` base, `EndEffectorKinematics`, rotation/skew helpers), `ocs2_perceptive` (plane/terrain decomposition helpers for perceptive MPC).

### Robotic example layout (the pattern to follow)

Each robot under `ocs2_robotic_examples/` is split into two packages:
- `ocs2_<robot>` — the core library: an `*Interface` class that assembles the `OptimalControlProblem` (dynamics, cost, constraints) from a URDF + `.info` config, plus a pybinding module and a `setup.py`. Config lives in `config/mpc/task.info`. RobCoGen-generated rigid-body dynamics live in `include/ocs2_<robot>/generated/` (sourced from `.kindsl`); package-source path at runtime is resolved via a `package_path.h.in` → `configure_file` template.
- `ocs2_<robot>_ros` — ROS 2 nodes (one per solver, e.g. `BallbotSqpMpcNode.cpp`, `BallbotMpcMrtNode.cpp`) and `.launch.py` files. Launch files are paired per solver (`*_ddp`, `*_sqp`, `*_slp`, `*_mpc_mrt`).

Config is always a Boost property-tree `.info` file (see `ocs2_ballbot/config/mpc/task.info`); per-solver settings blocks (`ddp`, `slp`, `sqp`, `ipm`, ...) are read into the corresponding `*Settings`.

## Conventions

- **C++17**, enforced via [ocs2_core/cmake/ocs2_cxx_flags.cmake](ocs2_core/cmake/ocs2_cxx_flags.cmake): `-pthread`, `-Wfatal-errors`, OpenMP, forced Boost dynamic linking (`BOOST_ALL_DYN_LINK`, raised MPL vector/map limits). New libraries apply these via `target_compile_options(<target> PUBLIC ${OCS2_CXX_FLAGS})`.
- **CppAd autodiff**: cost/constraint/dynamics classes commonly have a `*CppAd` variant. The compiled CppAD-CG shared libraries are generated **at runtime** and cached in a build folder; example interfaces expose a `recompileLibraries` flag in `task.info` to force regeneration when the model changes.
- **Threading**: OpenMP + `ocs2::ThreadPool` for parallel per-node/integration work; solvers are designed for real-time MPC.
- **Package structure**: headers in `include/<pkg>/`, sources in `src/`, tests in `test/`, cmake helpers in `cmake/`. Each package `find_package`s its OCS2 deps, uses `ament_target_dependencies`/`ament_export_dependencies`, and installs headers + an export target so downstream packages can link it.
- Every source/header carries the BSD-3-Clause boilerplate; preserve it on new files.
- CI (`.github/workflows/ros-build-test.yml`) builds a Release+tests matrix and a Debug-build-only (up-to `ocs2_ros_interfaces`) matrix on `ros:jazzy-ros-base`.
