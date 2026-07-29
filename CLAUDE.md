# Development Workflow

The workflow below is the process you should follow when starting any new task:

1. Explore relevant code/files to be touched
1. Create an implementation plan
1. Write code and tests
1. Build: `cmake --preset amd64-debug && cmake --build --preset amd64-debug`
1. Test: `ctest --preset amd64-debug`
1. Format: `git ls-files '*.cpp' '*.hpp' | grep -vE '^(third-party|umaa-cpp)/' | xargs clang-format -i` (config: /workspace/projects/.clang-format)

# Coding Standards

Rules below are binding. **MUST**/**NEVER** are absolute. **PREFER** means: do this unless you can state a concrete reason not to, in which case ask first.

## Language & Types

- Builds as C++20 (the umaa-cpp SDK exports `cxx_std_20` as a PUBLIC requirement). Write C++17 idioms: MUST NOT introduce C++20-only features (concepts, ranges, `std::span`/`std::format`, spaceship, designated initializers) — pinning C++17 proper waits on the future SDK port.
- MUST use fixed-width integers from `<cstdint>` (`int32_t`, `uint64_t`, ...). Never bare `int`/`long`/`short`.
- MUST use the internal aliases `flt32_t` / `flt64_t`. Never bare `float`/`double`.
- MUST use `std::scoped_lock`. NEVER `std::lock_guard<std::mutex>`.
- MUST use `while`. NEVER `do { } while ()`.
- NEVER use `goto`.
- NEVER use `friend` classes/functions. (MISRA-required rule; deviations need explicit approval.)
- NEVER use anonymous namespaces. Shared helpers go in a dedicated utility `.hpp` with the implementation in a `.cpp`; single-use helpers become `static` functions in the `.cpp`.

## Memory

- MUST use smart pointers. NEVER `new`, `delete`, `malloc`, or `free`.
- Default to `std::unique_ptr`. Use `std::shared_ptr` only when an object is genuinely co-owned across multiple locations for its lifetime. Use `std::weak_ptr` to break cycles.
- Documented exception: ownership transfer into vendored APIs that take raw pointers (e.g. httplib's `new_task_queue`) — one-line comment + `NOLINT` at the site.

## Files & Layout

- Extensions MUST be `.hpp` and `.cpp`.
- One class per file; filename MUST match the class name. Extra types are allowed only when tightly coupled to it (RAII handles, internal config structs).
- Pure virtual interfaces live in `I<InterfaceName>.hpp`.
- Headers live under `include/autopilot/<group>/` (groups: core, config, guidance, safety, modes, umaa, vehicle); sources mirror the layout under `src/<group>/`. Include style: `#include "autopilot/<group>/Name.hpp"`. Tools use `tools/clients/` and `tools/monitors/` with `#include "clients/Name.hpp"`.
- Include guards: `AUTOPILOT_<GROUP>_<NAME>_HPP_` (tools: `AUTOPILOT_TOOLS_<SUB>_<NAME>_HPP_`).
- Do NOT add copyright headers; the standard header text is pending and will be applied in a dedicated pass.

## Comments

- NEVER write paragraph comments. Convey intent with variable and function names instead — comments rot, names get refactored.
- Inline comments only when necessary: a few words, one sentence maximum.
- NEVER write banner comments. Bad: `// ---------------- INPUT TESTING -----------------`
- Deferred/out-of-scope work MUST be marked:
  - `// TODO(@user): what is deferred and why` — one sentence up to a short paragraph, enough that any reader understands the situation.
  - Non-trivial items MUST reference a real GitLab issue: `// TODO(<projectname>-#<issuenum>): ...`

## Formatting

- clang-format is the single source of truth. Config lives at `/workspace/projects/.clang-format`. Do not fight it, do not hand-format around it.
- Google C++ Style Guide is the baseline for anything clang-format does not decide.

## CMake

- Sources MUST be listed explicitly. NEVER use `file(GLOB ...)` anywhere.
- Public headers live in `include/`, exposed with generator expressions:
  ```cmake
  target_include_directories(mylib PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
  ```
- PREFER shared libraries. Static + `-fPIC` only in the rare justified case.
- MUST define install targets alongside a `CMakePresets.json` supporting exactly:
  ```sh
  cmake --preset <arch>-<debug|release>
  cmake --build --preset <arch>-<debug|release>
  cmake --install build/
  ```
- Binary dir MUST be `build/` for every preset. NEVER add per-preset or custom build dirs — switching presets means deleting `build/` and starting clean.

## Testing (GTest / GMock)

- Every test body MUST be laid out as:
  ```cpp
  // GIVEN: x
  // WHEN: y
  // THEN: z
  ```
- NEVER use namespaces in unit tests. Use fully qualified names. Carve-out: file-scope type aliases for UMAA generated types (`using ModeCmdEnum = UMAA::MM::...;`) are allowed — the generated paths run past 100 characters.
- MUST use GMock rather than hand-rolled test apparatus. Carve-out: hand-rolled kinematic/physics simulators are permitted where a mock cannot express vehicle motion (planner/end-to-end test vehicles); pure canned-return fakes MUST be GMock.
- NEVER add getters/setters or widen visibility for testability. Inject dependencies so tests can pass in mocks or expected values.
- PREFER stateless functions — they are trivially testable.

## Design

- Apply established patterns where they fit. Common in this codebase: observer, state, strategy, factory / abstract factory, decorator, command, thread pool (architectural).
- Cycle/step functions vs. data-driven events: both are valid: pick per situation and be able to justify the choice.

## Docker

- MUST use multi-stage Dockerfiles: faster builds, leaner final layer.