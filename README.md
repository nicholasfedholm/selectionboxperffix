# Selection Box Perffix

Vintage Story 1.22.x code mod. A Harmony prefix patch that makes vanilla `EntityBehaviorSelectionBoxes` hit testing (`getHitIndex(Ray)`, `getHitIndex()`, `Entity.IntersectsRay`) allocation-free and faster, with bit-identical hit results. It matters most for mods with many multi-box entities, such as LoP dinosaurs.

## Why
Vanilla allocates about 3.2 KB per call for a 5-box entity and about 22 KB per call for a 35-box one (small `Vec3d`/`Vec4d`/`Ray` objects). The client calls `getHitIndex()` every frame for every entity within 24 blocks, so a herd produces tens of MB/s of garbage.

## What it does
- All math runs on doubles in locals plus cached `[ThreadStatic]` scratch arrays. The only allocation left is the `Vec3d` vanilla stores in `hitPosition` on a hit.
- Same arithmetic in the same order as vanilla. The base matrix is built once per call instead of once per box.
- Per-entity cache (`ConditionalWeakTable`) of box matrices, inverses and bounding spheres, keyed bit-exactly on every input vanilla reads.
- Conservative bounding-sphere early-outs. Singular, ill-conditioned or NaN boxes are never skipped.

## Results (offline bench, dotnet 10, real LoP shapes)
- Correctness: 3,600,000 rays, 729,176 hits, 0 mismatches against vanilla.
- Allocation: 3-22 KB per call down to 0-8 B (about 99.9% less).
- CPU: about 3-4x faster when every call sees a new animation frame, 8-13x when the frame is reused.

An in-game A/B has not been done yet.

## Config
`ModConfig/SelectionBoxPerffix.json`: `Enabled`, `UseCache`, `EarlyOut`, `VerifyMode` (also runs vanilla and logs `VERIFY MISMATCH`; slower than vanilla, for checking only), `LogStats`.

## Build
Set `VSDir` to a game install containing `VintagestoryAPI.dll`, `Mods/VSEssentials.dll` and `Lib/0Harmony.dll`, then `dotnet build -c Release`. The default is `%LOCALAPPDATA%\Programs\Optimum`. Zip the output DLL with `modinfo.json` to install.

## Risks
- Mirrors 1.22.7 exactly. Re-verify after a game update that changes `applyBoxTransform` or `Testintersection`. If the target methods disappear, the patch fails at startup instead of misbehaving.
- A mod that prefixes the same methods may be skipped by the prefix's `return false`.
- Vanilla's private `hitPositionAABBSpace` is no longer updated on the patched path.
