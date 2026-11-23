import numpy as np
from itu840 import kl

c = 299_792_458.0
print("=== 1. Wavelength ratio (claimed '~3,000x longer than 905 nm') ===")
lam_radar, lam_lidar = c / 77e9, 905e-9
print(f"77 GHz wavelength = {lam_radar*1e3:.3f} mm;  905 nm LiDAR = {lam_lidar*1e9:.0f} nm")
print(f"ratio = {lam_radar/lam_lidar:,.0f}x   (claim was ~3,000x)")

print("\n=== 2. Do fog droplets 'match' the 905 nm wavelength? size parameter x = 2*pi*r/lambda ===")
for d_um in (4, 10, 20):
    r = d_um * 1e-6 / 2
    print(f"droplet diameter {d_um:>2} um: x(905 nm) = {2*np.pi*r/lam_lidar:6.1f}   x(77 GHz) = {2*np.pi*r/lam_radar:.4f}")

print("\n=== 3. Extinction values: V = 3.912/alpha ===")
for a in (0.05, 0.25, 0.50, 0.55, 0.85):
    print(f"alpha = {a:4.2f} /m  ->  visibility = {3.912/a:6.1f} m")
print("Our model's fog levels:")
for name, V in (("clear", 20000), ("light", 2000), ("moderate", 500), ("dense", 100)):
    print(f"  {name:<8} V = {V:>6} m -> alpha = {3.912/V:.5f} /m")

print("\n=== 4. Round-trip fog loss in dB (ITU-R P.840, 0 C) ===")
bands = (("X 10 GHz", 10), ("K 24 GHz", 24), ("Ka 35 GHz", 35), ("77 GHz", 77), ("W 94 GHz", 94))
ranges_km = (0.25, 1, 3, 5, 9)
for M, label in ((0.5, "DENSE fog (M=0.5 g/m3, ~50 m vis)"), (0.05, "MODERATE fog (M=0.05 g/m3, ~300 m vis)")):
    print(f"\n{label}")
    print("band".ljust(11) + "".join(f"{r:>8.2f}km" for r in ranges_km))
    for bn, f in bands:
        k = kl(f, 273.15)
        print(bn.ljust(11) + "".join(f"{2*k*M*r:>10.1f}" for r in ranges_km))

print("\n=== 5. Radar-equation scaling from an automotive-class 77 GHz radar (assumed ref: 250 m on a 10 m2 target) ===")
R_ref, rcs_ref = 250.0, 10.0
for name, rcs in (("small GA aircraft (~1 m2)", 1.0), ("small UAS (~0.1 m2)", 0.1), ("tiny drone (~0.01 m2)", 0.01)):
    print(f"  {name:<28} detectable to ~{R_ref*(rcs/rcs_ref)**0.25:5.0f} m")
R1 = R_ref * (1.0 / rcs_ref) ** 0.25
for R_t in (2000, 5000, 9000):
    print(f"  reach {R_t/1000:.0f} km on a 1 m2 target: ~{40*np.log10(R_t/R1):.0f} dB more (Pt*G^2*integration), before fog/gas loss")
print("Our simulation's radar model assumes max range = 12000 m")
