"""
fog.py
Environmental visibility degradation model for the Falcon Eye LiDAR-like
sensor.

Key research caveat (per spec section 6): dense fog is NOT assumed to be
"seen through" by LiDAR. Instead we model physically-motivated attenuation
of the optical return signal using a Beer-Lambert extinction law, and let
that attenuation drive down detection probability / usable range while
driving up noise and false-return rates. This lets the simulation honestly
answer "does the sensor still help in fog?" rather than assuming it does.
"""

from dataclasses import dataclass
import numpy as np


def angstrom_exponent(visibility_km: float, model: str) -> float:
    """Wavelength-dependence exponent q in sigma(lambda) = sigma_550nm * (lambda/550nm)^(-q).

    Two widely used empirical size-distribution models (atmospheric optics / free-space optics):
      * 'kruse': q = 1.6 (V>=50 km), 1.3 (6<=V<50), 0.585*V^(1/3) (V<6 km)
      * 'kim'  : q = 1.6 (V>50), 1.3 (6..50), 0.16V+0.34 (1..6), V-0.5 (0.5..1), 0 (V<0.5 km)
    Kim is generally preferred for low visibility (dense fog is nearly wavelength-independent).
    These are empirical fits, not guarantees for a specific LiDAR: calibrate against measured
    data (see falcon_eye.calibrate.fit_extinction_scale)."""
    v = max(float(visibility_km), 1e-3)
    if model == "kruse":
        if v >= 50.0: return 1.6
        if v >= 6.0: return 1.3
        return 0.585 * v ** (1.0 / 3.0)
    if model == "kim":
        if v > 50.0: return 1.6
        if v >= 6.0: return 1.3
        if v >= 1.0: return 0.16 * v + 0.34
        if v >= 0.5: return v - 0.5
        return 0.0
    raise ValueError(f"unknown extinction model '{model}' (use 'koschmieder', 'kruse' or 'kim')")


@dataclass
class FogScenario:
    name: str
    # Meteorological visibility, i.e. distance at which a large dark
    # object is just distinguishable (m). This is the standard variable
    # used to derive Beer-Lambert extinction coefficients.
    visibility_m: float
    # Extra multiplicative noise-inflation and false-return factors,
    # calibrated qualitatively (fog scatters returns -> more spurious hits).
    noise_multiplier: float
    false_return_multiplier: float
    # --- optional physics refinements (defaults reproduce the original behaviour exactly) ---
    wavelength_nm: float = 550.0            # wavelength at which transmittance is evaluated
    extinction_model: str = "koschmieder"   # 'koschmieder' (wavelength-independent) | 'kruse' | 'kim'
    extinction_scale: float = 1.0           # empirical multiplier on extinction (fitted from real data)

    def extinction_coefficient(self, wavelength_nm=None) -> float:
        """Koschmieder approximation: sigma = 3.912 / visibility (at 550 nm).
        Standard relation used in atmospheric optics / LiDAR range
        equations to relate meteorological visibility to the volume
        extinction coefficient (units: 1/m). With extinction_model 'kruse'/'kim' the value is
        rescaled to `wavelength_nm` (default: this scenario's wavelength_nm)."""
        base = 3.912 / max(self.visibility_m, 1.0)
        wl = self.wavelength_nm if wavelength_nm is None else float(wavelength_nm)
        if self.extinction_model != "koschmieder" and abs(wl - 550.0) > 1e-9:
            q = angstrom_exponent(self.visibility_m / 1000.0, self.extinction_model)
            base *= (wl / 550.0) ** (-q)
        return base * self.extinction_scale

    def two_way_transmittance(self, range_m: float, wavelength_nm=None) -> float:
        """Beer-Lambert two-way (out-and-back) atmospheric transmittance
        of the laser pulse at a given range. Value in [0, 1]."""
        sigma = self.extinction_coefficient(wavelength_nm)
        return float(np.exp(-2.0 * sigma * range_m))


@dataclass
class LayeredFogScenario(FogScenario):
    """Fog that is NOT homogeneous along the line of sight (e.g. a fog bank the target sits in).

    `layers` is a list of (r_start_m, r_end_m, visibility_m) segments measured from the sensor;
    outside every layer the base `visibility_m` applies. Transmittance integrates extinction
    along the path: T = exp(-2 * integral sigma(r) dr). `noise_multiplier` and
    `false_return_multiplier` stay scalar (they describe near-field scattering at the sensor)."""
    layers: tuple = ()

    def two_way_transmittance(self, range_m: float, wavelength_nm=None) -> float:
        wl = self.wavelength_nm if wavelength_nm is None else float(wavelength_nm)

        def ext(v):
            return FogScenario(self.name, v, 1.0, 1.0, wl, self.extinction_model,
                               self.extinction_scale).extinction_coefficient(wl)

        edges = {0.0, float(range_m)}
        for a, b, _ in self.layers:
            for e in (a, b):
                if 0.0 < e < range_m:
                    edges.add(float(e))
        pts = sorted(edges)
        total = 0.0
        for a, b in zip(pts[:-1], pts[1:]):
            mid = 0.5 * (a + b)
            v = self.visibility_m
            for la, lb, lv in self.layers:      # last matching layer wins
                if la <= mid < lb:
                    v = lv
            total += ext(v) * (b - a)
        return float(np.exp(-2.0 * total))


# Predefined scenarios per spec section 6
FOG_SCENARIOS = {
    "clear":    FogScenario("Clear air",     visibility_m=20000.0, noise_multiplier=1.0, false_return_multiplier=1.0),
    "light":    FogScenario("Light fog",     visibility_m=2000.0,  noise_multiplier=1.5, false_return_multiplier=1.8),
    "moderate": FogScenario("Moderate fog",  visibility_m=500.0,   noise_multiplier=2.5, false_return_multiplier=3.5),
    "dense":    FogScenario("Dense fog",     visibility_m=100.0,   noise_multiplier=5.0, false_return_multiplier=8.0),
}


def get_fog_scenario(key: str) -> FogScenario:
    if key not in FOG_SCENARIOS:
        raise ValueError(f"Unknown fog scenario '{key}'. Options: {list(FOG_SCENARIOS)}")
    return FOG_SCENARIOS[key]
