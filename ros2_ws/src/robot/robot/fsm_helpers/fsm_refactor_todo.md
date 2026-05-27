# MissionFSM Refactoring TODO (Task-Class Approach)

This document outlines the steps to refactor the monolithic `MissionFSM` into a modular, Task-Class based architecture.

## Phase 1: Define the Base Interface
- [ ] **Create `robot/fsm_helpers/task.py`**
  - Define the `Task` base class.
  - Required methods:
    - `__init__(self, robot)`: Store robot reference.
    - `update(self)`: Called every FSM tick.
    - `is_done(self) -> bool`: Returns `True` when the task is complete.
    - `on_enter(self)` (Optional): Called when the task first starts.
    - `on_exit(self)` (Optional): Called when the task finishes or is aborted.

## Phase 2: Implement Concrete Task Classes
Create individual classes for each distinct behavior currently handled in `MissionFSM`.

- [ ] **Create `NavTask` (in `task.py` or a new file like `tasks_nav.py`)**
  - Move logic from `_handle_nav()`.
  - Handle waypoint segmentation, path planners (`pp` vs `lapf`), and internal `NavStage` transitions.
  - Implement `is_done()` based on navigation handles completion.
- [ ] **Create `WaitTask`**
  - Move logic from `_handle_wait()`.
  - Handle triggers like `"green_light"` or button presses.
- [ ] **Create `ManipTask`** (if manipulation is implemented)
  - Encapsulate `PICK_SEQUENCE` and `PLACE_SEQUENCE` execution.
- [ ] **Create `IdentTask` / `PlanTask`** (if applicable based on `task_planner.py`)
  - Create stubs or implementations for these states.

## Phase 3: Update the Task Planner / Factory
We need a way to convert the dictionaries in `task_planner.py` into our new Task objects.

- [ ] **Create a Task Factory (in `fsm.py` or `task.py`)**
  - Write a function (e.g., `build_task(robot, task_dict) -> Task`) that takes a dictionary from `task_planner.py` and returns instantiated `NavTask`, `WaitTask`, etc.
  - *Alternatively*, modify `task_planner.py` to instantiate the classes directly, though keeping dictionaries is often cleaner for configuration.

## Phase 4: Refactor MissionFSM
Gut the internal state management from the main FSM and delegate to the active task.

- [ ] **Clean up `MissionFSM.__init__`**
  - Remove task-specific state variables (`nav_stage`, `nav_waypoints_idx`, `manip_sequence`, etc.).
  - Keep a `self.current_task: Task = None` and a way to track progress through the list.
- [ ] **Update `MissionFSM.on_enter("EXECUTE")`**
  - Instead of setting up `nav_init`, simply instantiate the current task using the factory and optionally call `self.current_task.on_enter()`.
- [ ] **Update `MissionFSM.update()` for the "EXECUTE" state**
  - Replace the monolithic `if/elif` chain with:
    ```python
    if self.current_task:
        self.current_task.update()
        if self.current_task.is_done():
            self.trigger("next")
    ```
- [ ] **Update `MissionFSM._advance_task()`**
  - Call `self.current_task.on_exit()` (if implemented) before advancing.
  - Set up the next task.
- [ ] **Remove obsolete helper methods**
  - Delete `_handle_nav`, `_handle_wait`, `_handle_manip`, and `_finish_segment` from `MissionFSM`.

## Phase 5: Testing and Validation
- [ ] **Test isolated tasks**
  - Write simple test scripts to run a `NavTask` standalone without the FSM.
- [ ] **Test full integration**
  - Run the `TEST_NAV` and `TEST_NAV2` modes on the robot to verify the sequence executes correctly.
- [ ] **Verify logging**
  - Ensure the SQLite logging in `MissionFSM` still captures the correct high-level state and task index. (You may want tasks to optionally log their internal stages if needed).
