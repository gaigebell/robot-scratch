# robot-scratch Project Instructions

## Goal

Build a modular robot experimentation infrastructure for heterogeneous
robots, teleoperation devices, sensors, policies, data collection, replay,
and simulation.

The current architecture source of truth is `docs/architecture.md`.
The current implementation state is recorded in `docs/project-status.md`.

## Working principles

- Do not claim hardware support without an independently reproducible test.
- Keep hardware drivers separate from mappings, orchestration, data recording,
  and user applications.
- User applications must not send raw commands directly to hardware SDKs.
- Every physical command must pass ownership arbitration and safety checks.
- Preserve raw timestamps, commands, feedback, observations, and safety events.
- Prefer small vertical slices with fake-device tests before real-hardware work.
- Never run real-robot commands unless the user explicitly requests it.
- Do not silently select coordinate transforms, units, or cross-embodiment mappings.
- Keep contracts versioned and document breaking changes.

## Development workflow

For substantial changes:

1. Read `docs/architecture.md` and `docs/project-status.md`.
2. Inspect the existing implementation and tests.
3. Create or update an implementation plan under `plans/`.
4. Implement the smallest end-to-end usable slice.
5. Run unit, integration, and configuration-validation tests.
6. Update `docs/project-status.md` and relevant ADRs.

Do not attempt to implement the entire architecture in one change.

## Agent context and handoff

- For a new task or handoff, use `docs/agent-onboarding.md` as the entry point;
  read additional research and ADRs only when relevant to the task.
- Record scope, acceptance criteria, decisions and a resumable checkpoint in
  the task plan. Templates are under `plans/templates/`.
- The M1 fake baseline is complete; consult `docs/project-status.md` before
  starting work. If the maintainer's local `plans/001-fake-control-loop.md`
  exists, consult its completion status as historical context as well.
  A plan or template does not authorize physical robot commands.
- When parallel tasks are explicitly requested, use separate checkouts and a
  common Git baseline. Coordinate shared contracts and global status updates
  through one integration task; do not overwrite another task's local work.
- Report validation level, environment and evidence. Unrun Linux/hardware tests
  remain unvalidated even when Windows fake tests pass.

## Public repository boundary

- Canonical project/distribution/CLI name: `robot-scratch`. Preserve the
  `robot_infra` import namespace, `robot-infra` CLI alias and versioned protocol
  identifiers unless a separate compatibility change is explicitly planned.
- Follow `docs/development-guide.md` and run `tools/check_project.py` before
  finalizing substantial changes.
- Use `publication.toml` as the public file allowlist; inspect staged bytes with
  `tools/check_publication.py --staged`. Never publish private research, device
  identities, local profiles, raw results, datasets, credentials or reference
  repositories. Keep them in ignored local directories.
- Git history is part of publication. Do not push private ancestor commits,
  all local branches or all tags when publishing a reviewed public branch.
