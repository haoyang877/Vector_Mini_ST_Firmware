# Plans and Decisions

- [`active/`](active/): work in progress with acceptance criteria and a decision log.
- [`completed/`](completed/): finished plans retained as design history.
- [`decisions/`](decisions/): durable architecture and workflow decisions.
- [`tech-debt-tracker.md`](tech-debt-tracker.md): acknowledged debt with an exit condition.

Small, local edits do not need a plan. Cross-module, protocol, release, safety, or harness
changes do. Completed plans must record the verification evidence used for acceptance.

Behavior-preserving refactor plans additionally record:

- the structural problem and measurable improvement target;
- unchanged external behavior, including protocol/parameter ABI, defaults, state transitions,
  fault behavior, control outputs, timing, stack, ROM/RAM, and artifacts as applicable;
- the pre-refactor test, golden-vector, timing, resource, and image/map baseline;
- staged migration steps, rollback point, and removal conditions for any temporary path;
- acceptance results, failed checks, unresolved debt, and the final `summary.json` evidence.
