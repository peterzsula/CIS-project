# Control and Intelligent Systems project

This branch includes the current colleagues' experiments, startup recovery,
and the local numbered model copies. Read [the system guide](docs/SYSTEM_GUIDE.md)
for the equations, controller logic, file map, experiments, and interpretation
of the plots. Read [the validation record](docs/VALIDATION.md) for what was tested.

## Run in MATLAB

Open this repository folder as MATLAB's current folder and run a whole script:

```matlab
setup_cartpole       % Part 2: optimize, simulate the cart-pole, and plot results
pendulum_script      % Part 1: simulate the controller and observer with noise
```

The recommended models are `cartpole.slx` and `actuated_pendulum.slx`.
`cartpole1.slx`, `cartpole2.slx`, and `actuated_pendulum1.slx` are retained legacy
copies. They now initialize successfully, but their older controller/noise
wiring does not include every feature in the canonical models. In particular,
the current cart-pole model receives `capture` from the setup; the older
numbered models retain fixed capture thresholds inside their controller.

For the model in the reported screenshot, this also works directly:

```matlab
open_system('cartpole2')
out = sim('cartpole2');
```

## Startup recovery

Every model's `InitFcn` calls `ensure_project_initialized`. If MATLAB has lost
the variables or physical structure fields, or the workspace belongs to the
other project part, it runs `initialize_cartpole` or `initialize_pendulum`.
These files initialize parameters without simulating or plotting. This fixes
errors such as missing `param.L`, `param.p0`, `param.bc`, `p`, and `Ts`.

Complete settings for the current part are preserved, so experiment sweeps
keep their custom gains, initial conditions, noise, and limits. A recovery
from missing data reloads that part's defaults. To keep a parameter change
across MATLAB restarts, save it in the corresponding initializer or experiment
script. Recompute iLQR after changing the Part 2 model, sample time, horizon,
or cost weights.

`pendulum_setup` contains the shared Part 1 controller/observer design;
individual experiments add their own conditions. `pendulum_script_original`
is the untouched original template.

## Colleagues' Part 1 experiments

Run any of these as a whole script:

```matlab
simscape_open_loop
lqr_evaluation
qr_sweep
sampling_time
actuator_limits
observer_poles
measurement_noise
input_disturbance
model_uncertainty
alternative_estimation
```

They print numerical results and export figures in this folder. Some test
conditions deliberately fail to stabilize; `ok = 0` or a `NaN` settling time
can be an experiment result. Generated figures and simulation caches are
ignored by Git. The long uncertainty sweeps include 300-second simulations.

`ode_script` additionally compares the nonlinear pendulum equation and its
upright linearization without running Simulink.

## Verify or repair after replacing models

```matlab
report = verify_project;          % startup and baseline behavior checks
report = verify_project(true);    % also execute all ten experiment scripts
repair_project_models            % reinstall callbacks in replaced model files
```

Validation writes `output/validation/latest.log` and `latest.json`.
Repairing models changes their files; use it after installing a copy that
lacks the callback, then review and commit those changes on your branch.

## MATLAB versions and Git

Tested with MATLAB R2026b, Simulink, Control System Toolbox, Simscape, and
Simscape Multibody. The canonical models and `actuated_pendulum1` are stored
in **R2026a format**; `cartpole1` and `cartpole2` are stored in **R2023b format**.
The repair uses MathWorks' [export to previous version](https://www.mathworks.com/help/simulink/slref/simulink.exporttoversion.html)
instead of leaving every model saved in R2026b. Execution in older installed
MATLAB releases has not been tested here.

R2026b may display an informational notification that a model was exported
to a previous version. This is expected for the shared files. If you edit
and save a model in R2026b, run `repair_project_models` before committing to
export it back to the shared format.

This work is on `Luca-branch`, based on the pulled `master` revision `4cf6494`.
Stay on that branch to retain these fixes:

```bash
git fetch origin
git switch Luca-branch
git pull --ff-only
```

Pulling `master` does not include changes that exist only on `Luca-branch`.
The model and scripts must come from the same revision. Keep all companion
initializers in the folder when copying a model to another machine.

## Current performance limit

The default Part 2 simulation balances upright, but the measured cart travel
briefly reaches about **1.099 m**, beyond the configured **1.0 m** target.
Its planned trajectory peaks at about **0.585 m**. The launcher now reports
the measured travel explicitly. The startup repair preserves the existing
controller rather than retuning its transient or claiming a hard travel
constraint has been enforced.
