import numpy as np

def kl(f_ghz, T_kelvin):
    """ITU-R P.840 cloud/fog specific attenuation coefficient, (dB/km)/(g/m^3).
    Rayleigh double-Debye model of liquid water permittivity."""
    th = 300.0 / T_kelvin
    e0 = 77.66 + 103.3 * (th - 1)
    e1 = 0.0671 * e0
    e2 = 3.52
    fp = 20.20 - 146.0 * (th - 1) + 316.0 * (th - 1) ** 2   # GHz
    fs = 39.8 * fp                                           # GHz
    f = f_ghz
    epp = f * (e0 - e1) / (fp * (1 + (f / fp) ** 2)) + f * (e1 - e2) / (fs * (1 + (f / fs) ** 2))
    ep = (e0 - e1) / (1 + (f / fp) ** 2) + (e1 - e2) / (1 + (f / fs) ** 2) + e2
    eta = (2 + ep) / epp
    return 0.819 * f / (epp * (1 + eta ** 2))
