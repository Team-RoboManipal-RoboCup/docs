# Task Planning & Execution

The Global Planner module for RoboManipal's RoboCup SML/EAI-WS 2026 entry is responsible for translating referee-issued task descriptions into an ordered, physically grounded action sequence for the warehouse robot. It sits between the referee order server and the execution layer, consuming `sml_messages/Task` on `/task` and publishing a fully resolved `sml_messages/PlannedTask` on `/planned_task` that downstream executor and arm nodes can walk step-by-step without any re-derivation.

---

## 1. Overview & Objectives

The planner operates on two order types defined by the EAI-WS rulebook:

- **Produce** (`OT_PRODUCE = 1`): fetch raw material blocks from storage shelves, transport to workbench for assembly, deliver finished product to customer counter.
- **Recycle** (`OT_RECYCLE = 2`): collect an assembled product from the customer counter, bring it to the workbench for disassembly, and return the constituent blocks to storage.

The primary performance objective is to **maximise the number of completed orders** within the run time. Speed per order is secondary; the planner therefore prioritises feasibility filtering, recycle-before-produce ordering (to restore inventory and unlock blocked produce orders), and shortest-product-first scheduling among produce orders.

### Why pyperplan (with fast-downward fallback)?

Several classical planners were evaluated for integration with the [Unified Planning (UP) framework](https://github.com/aiplan4eu/unified-planning):

| Planner | Strengths | Why not primary |
| :--- | :--- | :--- |
| **pyperplan** | Pure Python, zero native dependencies, instant startup, ships with `up-pyperplan` | Slower on large state spaces |
| **fast-downward** | Industry-strength, handles large grounded problems well | Requires compiled binary, slower cold start |
| **ENHSP** | Supports numeric fluents | Overkill; competition problems are propositional |
| **OPTIC / Temporal planners** | Parallel actions | EAI-WS tasks are strictly sequential |

pyperplan is the default because the state space for a single competition run (≤ 5 produce + 5 recycle orders, ≤ 5 blocks per product) remains tractable, startup is instantaneous inside the ROS 2 node, and there are no native library dependencies that could break on the competition machine. fast-downward is tried automatically if pyperplan returns no plan.

---

## 2. Technical Specifications

### ROS 2 Interface

| Direction | Topic | Message Type | Description |
| :--- | :--- | :--- | :--- |
| **Subscribe** | `/task` | `sml_messages/msg/Task` | Referee order + arena layout |
| **Publish** | `/planned_task` | `sml_messages/msg/PlannedTask` | Resolved action sequence |

### Downstream Interfaces (consumed by executor)

| Direction | Topic / Server | Type | Description |
| :--- | :--- | :--- | :--- |
| Subscribe | `/planned_task` | `sml_messages/msg/PlannedTask` | Executor reads steps |
| Publish | `/go_to_shelf_cmd` | `std_msgs/msg/String` | Station name to nav node |
| Subscribe | `/go_to_shelf_result` | `std_msgs/msg/String` | Nav ACK (`<station>:success`) |
| Action client | `/arm_command` | `sml_messages/action/ArmCommand` | Pick+place goal to arm |
| Service client | `/perception/get_averaged_pose` | `sml_messages/srv/PerceptionSrv` | Block pick pose at shelf |
| Service client | `/perception/get_averaged_place_pose` | `sml_messages/srv/PlacePosition` | Block place pose at workbench |

### Software Environment

| Parameter | Value |
| :--- | :--- |
| ROS 2 distro | Humble |
| Python | 3.10 |
| Planning library | `unified-planning` + `up-pyperplan`, `up-fast-downward` |
| Package | `robocup_planner` |
| Node name | `task_planner_node` |
| Executor node | `temp_warehouse_node` (temporary integration) |
| Nav node | `go_to_shelf` |
| Arm action server | `/arm_command` |

### `PlanStep.msg` Constants

| Constant | Value | Physical meaning |
| :--- | :--- | :--- |
| `FETCH_BLOCK` | 1 | Go to shelf, arm picks block, places on bot base |
| `GOTO_WORKBENCH` | 2 | Drive to workbench, arm places all base blocks onto workbench |
| `LOAD_TOY` | 3 | Arm picks assembled toy from workbench, places on bot base |
| `DELIVER` | 4 | Drive to customer counter, arm places toy |
| `FETCH_RECYCLE` | 5 | Drive to counter, arm picks assembled product, places on base |
| `PLACE_RECYCLE` | 6 | Drive to workbench, arm places product, wait for disassembly |
| `LOAD_BLOCKS` | 7 | Arm picks each disassembled block from workbench onto base |
| `RETURN_BLOCK` | 8 | Drive to storage shelf, arm places block from base onto shelf |

### `ArmCommand.action` Definition

```
# Goal — one complete pick-and-place cycle
string  object_id
float32 pick_x
float32 pick_y
float32 pick_z
float32 place_x
float32 place_y
float32 place_z
---
# Result
bool success
---
# Feedback
string status
```

### `PerceptionSrv.srv` Definition

```
# Request
int32 target_samples
---
# Response
geometry_msgs/PoseArray         block_poses
string[]                        block_ids      # int as string; "-1" = ID failure
string[]                        block_classes  # e.g. "4x2_red"
shape_msgs/SolidPrimitive[]     block_dims
```

---

## 3. Design & Architecture

### 3.1 Pipeline

```
Referee order server
        │  sml_messages/Task  (/task)
        ▼
┌───────────────────┐
│  TaskPlannerNode  │  unified_planning (pyperplan / fast-downward)
│  robocup_planner  │
└───────────────────┘
        │  sml_messages/PlannedTask  (/planned_task)
        ▼
┌────────────────────────┐
│  TempWarehouseNode     │  walks PlanStep[] sequentially
│  (executor + arm ctrl) │
└────────────────────────┘
        │                           │
        │ std_msgs/String            │ sml_messages/action/ArmCommand
        ▼                           ▼
  go_to_shelf (nav)           /arm_command (arm action server)
        │
        │ /go_to_shelf_result
        └──────────────────► TempWarehouseNode (ACK)
```

```{figure} assets/task_msg_structure.jpg
:alt: Task message format received from referee server
:width: 75%

Task message format received from referee server.
```

### 3.2 Unified Planning Problem Structure

The UP problem is rebuilt from scratch on every `/task` callback. All per-order actions are **fully grounded** (no free parameters) to guarantee pyperplan can solve them without negation in preconditions.

#### Types

| UP Type | Represents |
| :--- | :--- |
| `Block` | One raw material block object |
| `Toy` | One assembled product per order |
| `OrderObj` | One order object (produce or recycle) |

#### Fluents

| Fluent | Arity | Meaning |
| :--- | :--- | :--- |
| `block_on_shelf(block)` | 1 | Block is at a storage station |
| `block_on_base(block)` | 1 | Block is loaded on the bot base |
| `block_on_workbench(block)` | 1 | Block is placed on the workbench |
| `toy_at_counter(toy)` | 1 | Assembled toy is at customer counter |
| `toy_on_base(toy)` | 1 | Toy is on the bot base |
| `toy_on_workbench(toy)` | 1 | Toy is on the workbench |
| `toy_assembled(toy)` | 1 | Toy has been assembled |
| `toy_broken_down(toy)` | 1 | Toy has been disassembled |
| `arm_free` | 0 | Arm mutex — prevents concurrent arm actions |
| `robot_free` | 0 | Bot mutex — prevents nav during arm ops |
| `all_blocks_on_base(order)` | 1 | Gate: all blocks for this order are on base |
| `all_blocks_on_wb(order)` | 1 | Gate: all blocks placed on workbench |
| `all_blocks_loaded(order)` | 1 | Gate: all disassembled blocks on base (recycle) |
| `order_done(order)` | 1 | Order fully completed |
| `recycle_phase_done` | 0 | All recycle orders finished; produce may start |

#### Grounded Gate Actions (produce, per order)

```
set_all_on_base_N()
  PRE:  block_on_base(b) for all b in order, arm_free
  EFF:  all_blocks_on_base(order_N) = True

goto_workbench_N()
  PRE:  all_blocks_on_base(order_N), arm_free
  EFF:  block_on_workbench(b) for all b,
        all_blocks_on_wb(order_N) = True,
        toy_assembled(toy_N) = True,
        toy_on_workbench(toy_N) = True

load_toy_N()
  PRE:  toy_on_workbench(toy_N), toy_assembled(toy_N), arm_free
  EFF:  toy_on_base(toy_N) = True

deliver_N()
  PRE:  toy_on_base(toy_N), arm_free
  EFF:  toy_at_counter(toy_N) = True, order_done(order_N) = True
```

#### Grounded Gate Actions (recycle, per order)

```
fetch_recycle_N()
  PRE:  toy_at_counter(toy_N), arm_free, robot_free
  EFF:  toy_on_base(toy_N) = True

place_recycle_N()
  PRE:  toy_on_base(toy_N), arm_free
  EFF:  toy_broken_down(toy_N) = True,
        block_on_workbench(b) for all b in order

load_blocks_N()
  PRE:  toy_broken_down(toy_N), block_on_workbench(b) for all b, arm_free
  EFF:  block_on_base(b) for all b,
        all_blocks_loaded(order_N) = True

return_block_oN_bM_tT()      # one per block, fully grounded
  PRE:  block_on_base(blk), arm_free, robot_free, all_blocks_loaded(order_N)
  EFF:  block_on_shelf(blk) = True

recycle_done_N()
  PRE:  block_on_shelf(b) for all b
  EFF:  order_done(order_N) = True
        [last recycle only] recycle_phase_done = True
```

### 3.3 Order Prioritisation

```python
# Recycle always before produce (restores inventory for blocked produce orders)
all_feasible = feasible_recycle + sorted(feasible_produce,
                   key=lambda o: len(decode_product(o.product_id)))
```

Feasibility for produce requires all needed block types present in `ST_STORAGE` stations. Feasibility for recycle requires the product's block types present in `ST_CUSTOMER` or `ST_HYBRID` stations.

### 3.4 Recycle Return Shelf Assignment

Disassembled blocks are not returned to their original shelf. Instead, the planner assigns each block to the storage shelf with the **fewest current occupants**, updating the count after each assignment to spread load across shelves:

```python
occ = dict(shelf_occupancy)        # {shelf_name: current_block_count}
for bi, bt in enumerate(seq):
    shelf = min(occ, key=occ.get)  # least occupied
    ret_shelf_by_idx[bi] = shelf
    occ[shelf] += 1                # update so next block goes elsewhere
```

Block index (`bi`) is used as the dict key instead of block type to correctly handle products that contain duplicate block types (e.g. product `33` needs two `type3` blocks).

### 3.5 Product ID Decoding

Product IDs encode the raw material sequence directly as decimal digits read bottom-to-top:

$$
\text{product\_id} = 341 \Rightarrow [3, 4, 1]
$$

where digit position 0 (leftmost) is the bottom block and the last digit is the top block.

```{figure} assets/product_assembly_catalog.jpg
:alt: Product assembly catalog
:width: 80%

EAI-WS raw material IDs and product assembly structures. Product ID encodes block sequence bottom-to-top.
```

```python
def decode_product(product_id: int) -> list[int]:
    return [int(d) for d in str(product_id)]
```

### 3.6 Perception Cache (TempWarehouseNode)

The bot can only determine block positions after arriving at a station. To avoid redundant service calls when multiple blocks are fetched from the same shelf, the executor caches perception results per station:

```
On arrival at station S (first visit):
  call /perception/get_averaged_pose(target_samples=N)
  store response as pose_cache[S] = {block_type: [(x,y,z), ...]}

On subsequent FETCH_BLOCK at same station S:
  pop pose from pose_cache[S][block_type]
  (no nav, no perception call)

On departure from S (navigating to different station):
  del pose_cache[S]   # prevent stale poses on re-entry
```

Pick coordinates come from `/perception/get_averaged_pose`; place coordinates on the workbench come from `/perception/get_averaged_place_pose`. Bot base place coordinates remain hardcoded slot-indexed (`BASE_SLOTS[0..2]`) pending perception support for the base surface.

---

## 4. Software / Hardware Interface

### 4.1 Topic & Service Summary

| Name | Type | Direction | Node |
| :--- | :--- | :--- | :--- |
| `/task` | `sml_messages/msg/Task` | In → Planner | Referee server |
| `/planned_task` | `sml_messages/msg/PlannedTask` | Planner → Executor | `task_planner_node` |
| `/go_to_shelf_cmd` | `std_msgs/msg/String` | Executor → Nav | `temp_warehouse_node` |
| `/go_to_shelf_result` | `std_msgs/msg/String` | Nav → Executor | `go_to_shelf` |
| `/arm_command` | `sml_messages/action/ArmCommand` | Executor → Arm | `temp_warehouse_node` |
| `/perception/get_averaged_pose` | `sml_messages/srv/PerceptionSrv` | Executor → Perception | pick pose |
| `/perception/get_averaged_place_pose` | `sml_messages/srv/PlacePosition` | Executor → Perception | workbench place pose |

### 4.2 Running the Stack

```bash
# 1. Build
cd ~/robocup_ws
colcon build --packages-select sml_messages robocup_planner
source install/setup.bash

# 2. Launch planner
ros2 run robocup_planner task_planner

# 3. Launch executor
ros2 run robocup_planner temp_warehouse

# 4. Launch nav node
ros2 run robocup_planner go_to_shelf

# 5. Inject a test task (produce Battery = product 34)
ros2 topic pub --once /task sml_messages/msg/Task "{
  order_list: [{order_type: 1, product_id: 34, name: 'battery_order'}],
  arena_layout: [
    {station_type: 1, station_id: 1, name: 's_top',        material_ids: [3]},
    {station_type: 1, station_id: 2, name: 's_center_mid', material_ids: [4]},
    {station_type: 3, station_id: 3, name: 'cc_1',         material_ids: []},
    {station_type: 2, station_id: 4, name: 'wb_top',        material_ids: []}
  ]
}"

# 6. Monitor planned steps
ros2 topic echo /planned_task
```

```{figure} assets/task_planner_execution_log.jpg
:alt: Task Planner output log
:width: 85%

Task Planner output showing UP solver resolving a 3-block produce order into a 6-step PlannedTask.
```

:::::{grid} 2

::::{grid-item}
:::{figure} assets/warehouse_exec_fetch.jpg
:alt: Warehouse fetch execution
:width: 100%

Warehouse execution: fetch from station.
:::
::::

::::{grid-item}
:::{figure} assets/warehouse_exec_deliver.jpg
:alt: Warehouse deliver execution
:width: 100%

Warehouse execution: deliver to counter.
:::
::::

:::::

### 4.3 Key Code Snippets

#### Solver with fallback

```python
solvers = ['pyperplan', 'fast-downward']
for solver in solvers:
    with OneshotPlanner(name=solver) as planner:
        result = planner.solve(problem)
        if result.plan is not None:
            break
```

#### Perception cache population

```python
cache: dict[int, list[tuple]] = {}
for pose, block_id_str in zip(response.block_poses.poses, response.block_ids):
    block_type = int(block_id_str)   # "-1" skipped
    if block_type == -1:
        continue
    cache.setdefault(block_type, []).append(
        (pose.position.x, pose.position.y, pose.position.z)
    )
```

#### Redundant nav suppression

```python
def _navigate_to(self, station: str) -> bool:
    if station == self._current_station:
        return True          # already parked here
    self._pose_cache.pop(self._current_station, None)   # clear stale cache
    ...
    self._current_station = station
    return True
```

### 4.4 Arena Context

The EAI-WS arena (10 m × 7.5 m) contains storage shelves, workbenches, and customer counters as enumerated stations. The referee populates `arena_layout` in the `/task` message with `station_type`, `station_id`, and `material_ids` for each active station. The planner reads this directly — no hardcoded arena map is required.

:::{note}
Station types: `ST_STORAGE = 1`, `ST_WORKBENCH = 2`, `ST_CUSTOMER = 3`, `ST_HYBRID = 4`. These constants are defined in `sml_messages/msg/Station.msg`.
:::

:::{warning}
`recycle_phase_done` is set to `True` only after the **last** recycle order's `recycle_done_N` action fires. If a recycle order is infeasible (missing blocks at counter), produce orders will be blocked indefinitely. Always verify `arena_layout` populates `material_ids` on `ST_CUSTOMER` stations for recycle orders.
:::

:::{tip}
To force pyperplan and skip the fast-downward fallback during testing, set `solvers = ['pyperplan']` in `_solve_with_up`. fast-downward requires a compiled binary on `$PATH` (`downward`).
:::
