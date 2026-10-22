# StarPilot + MoreTorque Torque Interceptor (Mazda)

This branch is upstream [StarPilot](https://github.com/firestar5683/StarPilot) with the Mazda
Torque Interceptor support from [MoreTore/openpilot](https://github.com/MoreTore/openpilot)
(`StarPilot-dev`) carried over as a patch. It targets the comma four and a 2023 Mazda 3
with a Torque Interceptor 2 (Gen2 platform), but keeps MoreTore's Gen1 and Gen3 code.

- Base: StarPilot `StarPilot` branch, commit `823a5d5` (2026-10-15)
- Mazda source: MoreTore `StarPilot-dev`, commit `c183c4c` (2026-09-12)
- License: MIT, same as both upstreams

## What the patch changes

| Area | Files |
| --- | --- |
| Mazda car port (Gen1/2/3, TI, radar interceptor) | `opendbc/car/mazda/*`, `opendbc/dbc/mazda_201{7,9}.dbc`, `mazda_2023.dbc` |
| Panda safety mode | `opendbc/safety/modes/mazda.h` |
| Panda firmware | `panda/board/main.c` (Gen1 TI CAN mode), `panda/board/drivers/can_common.h` (Gen2 ignition) |
| CAN checksums, torque tune, CX-8 | `opendbc/can/dbc.py`, `car/torque_data/*.toml`, `car/fingerprints.py` |
| Params and UI toggles | `common/params_keys.h`, mici and big-UI settings, `starpilot_state.py` |
| Blended ACC | `selfdrive/controls/lib/longcontrol.py` (gated to Mazda) |
| Tests | `safety/tests/test_mazda.py` (Gen1, Gen1+TI, new Gen2 class), `safety/tests/common.py`, `car/tests/routes.py` |

Changes from MoreTore's version: the blended-ACC PID reset only runs on Mazdas, the Mazda
toggles on the comma four only show when a Mazda is detected, memory params use
`Params(memory=True)`, and the Mazda safety tests were fixed and extended.

The captured firmware from the target 2023 Mazda 3 BP is included under
`MAZDA_3_2019` (GEN2): engine `PXNA-188K2-F`, transmission `PXLS-21PS1-A`,
and radar `B0N2-67XK2-E`, with the captured NUL padding. A regression test checks
that all six captured ECUs uniquely match this platform without a fingerprint override.
`MAZDA_3_2023` is a different, GEN3 platform; do not select it based on model year.
The torque-interceptor toggle controls GEN1; GEN2 uses its TI2 path directly.

## Building on the device

The `prebuilt` marker is removed, so the device compiles on boot. The first boot after
installing takes several minutes; later boots use the scons cache. This rebuilds the panda
firmware (where the Mazda safety code runs) and the params library (for the new toggles).

## Updating to a newer StarPilot

```bash
port_ref=$(git rev-parse HEAD)  # start with the completed port committed
patch_file=$(mktemp /tmp/mazda-ti2.XXXXXX.patch)
cp mazda_ti2/mazda-ti2.patch "$patch_file"
git remote add starpilot https://github.com/firestar5683/StarPilot.git
git fetch --depth 1 starpilot StarPilot
git checkout -b update starpilot/StarPilot
git apply --check "$patch_file"
git apply "$patch_file"   # if the check fails, adapt the patch to upstream first
git rm -q prebuilt
git restore --source="$port_ref" -- mazda_ti2
```

Add the `starpilot` remote only once. Run these commands from the repository root,
with a clean worktree. After adapting to a new base, regenerate the reusable patch
against that base, including the captured-firmware test, and update the base SHA above.

Then rerun the tests:

```bash
cd opendbc_repo && scons -j8
PYTHONPATH=$PWD:.. pytest opendbc/safety/tests/test_mazda.py
PYTHONPATH=$PWD:.. pytest opendbc/car/tests/test_car_interfaces.py -k MAZDA
PYTHONPATH=$PWD:.. pytest opendbc/car/tests/test_mazda_2023_fingerprint.py
PYTHONPATH=$PWD:.. pytest opendbc/car/tests/test_fw_fingerprint.py -k 'mazda or test_request_ecus_in_versions'
```

## Known gaps

Host validation of the car-specific completion: all six captured ECUs uniquely match
`MAZDA_3_2019`; the selected interface, fingerprint, and query-consistency checks pass
(44 tests, 10 skipped); the Mazda safety suite passes (74 tests, 16 skipped).
This does not establish a successful comma FOUR build, flash, or vehicle validation.

- **Gen2 longitudinal is not checked by the panda.** The safety mode lets openpilot send
  the `ACC` message (0x220) with any value, engaged or not. Openpilot passes the stock
  value through when it isn't in control, but the firmware does not enforce that.
- **Gen1 Torque Interceptor steering commands (0x249) are not torque-limited** in the
  safety mode; only Gen2/Gen3 check them.
- No CX-90 support. No public CI routes for the Gen2/Gen3 platforms.
- MISRA analysis was not run on `mazda.h` (cppcheck unavailable).
