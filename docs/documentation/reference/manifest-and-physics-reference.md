---
icon: table
---

# Manifest Schema, Physics Defaults & Troubleshooting

## Recommended `ValidationLimits` starting points

| Limit | Recommended CI Value | Meaning |
| --- | --- | --- |
| `maxTipRiskRatio` | `0.75 – 0.85` | `1.0` means inside wheel lift; `0.75` keeps a 25% safety margin |
| `maxTractionDemandRatio` | `0.90 – 1.00` | `> 1.0` means tires will saturate and skid on carpet |
| `maxTerrainTiltDeg` | `10.0° – 12.0°` | Maximum chassis pitch/roll allowed over bumps/ramps |

## Common troubleshooting

* **`PathTrajectoryValidator` fails at sample 100+ on a sharp corner?** Reduce `maxLinearAccelMps2` in your path constraints or lower CG height during transit.
* **Camera `Transform3d` rotated 90° in AdvantageScope?** Verify your Onshape Mate Connector has `+X` pointing out the lens and `+Z` pointing up (WPILib NWU convention).
