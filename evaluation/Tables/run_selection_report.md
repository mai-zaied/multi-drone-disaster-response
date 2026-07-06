# Run-selection report

## Tier comparison (scenario = medium)
- cloud/run_01 `cloud_medium_run_01.summary (1).json` -> **SELECTED** (score 9: events=4, completion_mean=2.8159)
- cloud/run_01 `cloud_medium_run_01.summary.json` -> **rejected** (score 0: events=0, completion_mean=None)
- fog/run_01 `fog_medium_run_01.summary (2).json` -> **SELECTED** (score 9: events=4, completion_mean=3.3051)
- fog/run_01 `fog_medium_run_01.summary (1).json` -> **rejected** (score 7: events=4, completion_mean=0.0017)
- local/run_01 `local_medium_run_01.summary.json` -> **SELECTED** (only candidate)

## Area scaling (fog / no-fault runs with coverage.area_m2)
- `fog_small_fog_small_01.summary.json` area=5,130 m2 dur=180.27s cov=97.55%
- `fog_small_fog_small_02.summary.json` area=9,600 m2 dur=264.11s cov=91.67%
- `fog_medium_run_01.summary (1).json` area=20,700 m2 dur=750.0s cov=96.55%
- `fog_large_fog_large_01.summary.json` area=30,360 m2 dur=1124.03s cov=86.64%

## Utilisation (normal fog)
- `fog_medium_run_01.summary (1).json` SELECTED

## Failure recovery
- `fog_medium_drone_fail_01.summary.json` SELECTED
