# MissionFSM Refactoring TODO (Task-Class Approach) - COMPLETED

This document outlines the steps to refactor the monolithic `MissionFSM` into a modular, Task-Class based architecture.

## Phase 1: Define the Base Interface
- [x] **Create `robot/fsm_helpers/task.py`**
  - Define the `Task` base class.
  - Required methods: `__init__`, `update`, `is_done`, `on_enter`, `on_exit`.

## Phase 2: Implement Concrete Task Classes
- [x] **Create `NavTask`**
  - Moved logic from `_handle_nav()`.
  - Handles waypoint segmentation, path planners (`pp` vs `lapf`), and internal `NavStage` transitions.
- [x] **Create `WaitTask`**
  - Moved logic from `_handle_wait()`.
  - Handles triggers like `"green_light"` or button presses.
- [x] **Create `ManipTask`**
  - Encapsulates `PICK_SEQUENCE` and `PLACE_SEQUENCE` execution.
- [x] **Create `IdentTask` / `PlanTask`**
  - Created stubs/implementations for these states.

## Phase 3: Update the Task Planner / Factory
- [x] **Create a Task Factory (in `task.py`)**
  - Implemented `build_task(robot, task_dict) -> Task`.

## Phase 4: Refactor MissionFSM
- [x] **Clean up `MissionFSM.__init__`**
  - Removed task-specific state variables.
  - Added `self.current_task: Task = None`.
- [x] **Update `MissionFSM.on_enter("EXECUTE")`**
  - Uses `build_task` factory to instantiate tasks.
- [x] **Update `MissionFSM.update()` for the "EXECUTE" state**
  - Delegates to `self.current_task.update()`.
- [x] **Update `MissionFSM._advance_task()`**
  - Calls `self.current_task.on_exit()` before advancing.
- [x] **Remove obsolete helper methods**
  - Deleted `_handle_nav`, `_handle_wait`, `_handle_manip`, etc. from `MissionFSM`.

## Phase 5: Testing and Validation
- [x] **Syntax check**
  - Verified `task.py` and `fsm.py` with `py_compile`.
- [ ] **Test full integration**
  - *Requires robot hardware/simulation.*
