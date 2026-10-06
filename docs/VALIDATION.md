# Validation record

The startup repair was tested on 6 October 2026 in MATLAB
26.2.0.3386108 (R2026b) with the installed Simulink, Control System Toolbox,
Simscape and Simscape Multibody products. Models were reloaded after export
to their shared formats before the simulation checks.

Run `verify_project(true)` to reproduce the integration and experiment checks.
The detailed generated log is `output/validation/latest.log`; the machine
readable result is `output/validation/latest.json`. These runtime outputs
are ignored by Git. The completed result is recorded below after the run.

The checks distinguish executable models and scripts from successful control
in every experiment condition. Several sweeps intentionally test unstable,
saturated, noisy or inaccurate-model scenarios. Their unsuccessful rows are
results to interpret, not missing-block errors.

The baseline cart-pole balances, but its measured peak travel exceeds the
configured 1.0 m target. That performance limitation is reported explicitly
and is not treated as a passing hard-constraint test.

## Completed results

**19 of 19 checks passed**, including execution of all ten Simulink experiment
scripts. The startup checks cover an empty Part 1 workspace, missing `p.l`,
preservation of custom experiment settings, the Part 2 launcher, the exact
`cartpole2` missing-`param` failure from the screenshot, the other numbered
models, and switching between the two parameter types/sample periods.

| Default run | Result |
| --- | --- |
| Part 1 launcher | 10-second simulation completed; angle settled within the reported 1-degree threshold after 2.12 s |
| Part 2 canonical model | Capture at 6.400 s; balanced during the final second of the 12.18-second run |
| `cartpole2`, with `param` deleted | Initialization recovered the physical fields; capture and balancing matched the canonical baseline |
| Part 2 final angle | Approximately -0.001113 degrees |
| Part 2 final cart position | Approximately -0.000058 m |
| Planned peak cart travel | Approximately 0.585 m |
| Measured peak cart travel | Approximately 1.099 m; exceeds the 1.0 m target |

Model archive checks confirmed saved callbacks and intact ZIP archives.
Every model retained its original block names and block types after export.
The canonical models and legacy pendulum copy remain R2026a files; the
numbered cart-pole copies remain R2023b files. Only R2026b execution was
available for testing.

The standalone `ode_script` also completed. A final check closed and reloaded
the repaired `cartpole2` file, deleted both `param` and `Ts`, and completed a
direct simulation. Thus recovery was verified from the saved model file as
well as the models already loaded in memory.
