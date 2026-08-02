# EMAFilter

Discrete-time exponential moving average (EMA) low-pass filter.

## What it is

A first-order IIR low-pass filter — the discrete-time equivalent of an
analog RC low-pass. `alpha` is recomputed every call from the actual sample
interval (`dt`) and a fixed time constant (`tau`), rather than using one
fixed `alpha`. This keeps the filter's behaviour consistent even when
`update()` isn't called at a perfectly fixed rate (e.g. inside a superloop
rather than a fixed-period ISR).

## Formula

```
alpha = dt / (tau + dt)
y[n]  = alpha * x[n] + (1 - alpha) * y[n-1]
```

Derived by discretising the continuous RC low-pass `tau*dy/dt + y = x(t)`
with a backward difference.

## API

| Method | Description |
|---|---|
| `EMAFilter(float timeConstant)` | Construct with time constant `tau` (seconds) |
| `float update(float rawValue, float dt)` | Feed one sample, returns filtered value |
| `void reset(float value = 0.0f)` | Reseed filter to a known value |
| `float getValue() const` | Last filtered output |
| `float getTimeConstant() const` | Current `tau` |
| `void setTimeConstant(float timeConstant)` | Change `tau` at runtime |

## Usage

```cpp
#include <EMAFilter.h>

EMAFilter filter(0.02f); // 20 ms time constant

void loop() {
    float raw = readNoisySignal();
    float dt  = computeDt();
    float smoothed = filter.update(raw, dt);
}
```

## Tuning tau

- Step response reaches **63%** of a step change after `1*tau`, **~99%** after `5*tau`.
- Cutoff frequency: `fc = 1 / (2*pi*tau)` — signals varying slower than `fc`
  pass through mostly unchanged; faster variation (noise) is attenuated.
- Larger `tau` → more noise rejection, more lag.
- Smaller `tau` → faster response, less smoothing.
- Pick the smallest `tau` that adequately suppresses your noise floor —
  extra smoothing beyond that only adds lag to anything downstream
  (e.g. a control loop) without further benefit.

## Notes

- The first `update()` call seeds the filter directly from `rawValue`
  instead of `0`, avoiding a startup transient.
- `reset(value)` reseeds to a known value; the next `update()` filters
  normally from there rather than jumping straight to the next raw sample.