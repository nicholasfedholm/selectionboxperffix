using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Vintagestory.API.Client;
using Vintagestory.API.Common;
using Vintagestory.API.Common.Entities;
using Vintagestory.API.MathTools;
using Vintagestory.GameContent;

namespace SelectionBoxPerffix
{
    /// <summary>
    /// Allocation-free re-implementation of EntityBehaviorSelectionBoxes.getHitIndex(Ray)
    /// (VSEssentials 1.22.7). Arithmetic is kept in the same order and precision as vanilla
    /// so the chosen box index and hit position are bit-identical.
    ///
    /// Speedups:
    ///  1. No per-call heap allocations (vanilla: ~6 Vec3d/Vec4d/Ray per box plus 2-4 per face test).
    ///  2. The entity-level part of applyBoxTransform (yaw, renderer tilt, swivel, size) is built once
    ///     per call instead of once per box.
    ///  3. Per-entity cache of box matrices/inverses, keyed bit-exactly on every input
    ///     (yaw, renderer angles, selection box height, size, and every box's AnimModelMatrix).
    ///  4. Conservative bounding-sphere early-outs (entity level and box level): a box is only
    ///     skipped when its parallelepiped provably cannot be hit by the half-line OR every hit point
    ///     would be beyond the ray length (vanilla rejects those too). Margins cover float error,
    ///     ill-conditioned / singular boxes are never skipped.
    /// </summary>
    public static class SelBoxCore
    {
        // ---- config (set once at startup) ----
        public static bool UseCache = true;
        public static bool EarlyOut = true;
        public static bool Stats = false;

        // ---- stats (non-atomic, approximate; only for logging) ----
        public static long Calls, EntityRejects, BoxTests, BoxSkips, CacheHits, CacheMisses;

        const double CondLimit = 1e4;

        sealed class BoxCache
        {
            public AttachmentPointAndPose[] arr;
            public int n;
            public float[] sizes;      // n*3
            public float[] animKey;    // n*16
            public float[] m;          // n*16 mvmat per box
            public float[] inv;        // n*16 inverse (lazy)
            public bool[] invOk;
            public double[] sph;       // n*4 cx,cy,cz,r+margin
            public bool[] sphOk;
            public bool entOk;
            public double ecx, ecy, ecz, er;
            public bool baseValid;
            public int kYaw, kXa, kYa, kZa, kSw, kY2, kSize;
            public object kEsr;
        }

        static readonly ConditionalWeakTable<EntityBehaviorSelectionBoxes, BoxCache> caches = new ConditionalWeakTable<EntityBehaviorSelectionBoxes, BoxCache>();

        [ThreadStatic] static float[] tBase;
        [ThreadStatic] static BoxCache tNoCache;

        static int Bits(float f) => BitConverter.SingleToInt32Bits(f);

        /// <summary>Drop-in for vanilla getHitIndex(Ray) given ray origin (entity-relative) and dir.
        /// Returns index or -1; on hit writes the AABB-space hit position (vanilla hitPositionAABBSpace).</summary>
        public static int GetHitIndex(EntityBehaviorSelectionBoxes bh, double ox, double oy, double oz, double dx, double dy, double dz, out double hx, out double hy, out double hz)
        {
            hx = hy = hz = 0;
            if (Stats) Calls++;
            var arr = bh.selectionBoxes;
            if (arr == null || arr.Length == 0) return -1;

            BoxCache c;
            if (UseCache)
            {
                c = caches.GetValue(bh, static _ => new BoxCache());
                lock (c) return Query(bh, c, arr, ox, oy, oz, dx, dy, dz, out hx, out hy, out hz);
            }
            else
            {
                c = tNoCache ??= new BoxCache();
                c.arr = null; // force full rebuild every call (thread-local scratch, no lock needed)
                return Query(bh, c, arr, ox, oy, oz, dx, dy, dz, out hx, out hy, out hz);
            }
        }

        static void EnsureArrays(BoxCache c, AttachmentPointAndPose[] arr)
        {
            int n = arr.Length;
            if (c.m == null || c.m.Length < n * 16)
            {
                c.sizes = new float[n * 3];
                c.animKey = new float[n * 16];
                c.m = new float[n * 16];
                c.inv = new float[n * 16];
                c.invOk = new bool[n];
                c.sph = new double[n * 4];
                c.sphOk = new bool[n];
            }
            c.n = n;
            c.arr = arr;
            for (int i = 0; i < n; i++)
            {
                var selem = arr[i].AttachPoint.ParentElement;
                c.sizes[i * 3 + 0] = (float)(selem.To[0] - selem.From[0]) / 16f;
                c.sizes[i * 3 + 1] = (float)(selem.To[1] - selem.From[1]) / 16f;
                c.sizes[i * 3 + 2] = (float)(selem.To[2] - selem.From[2]) / 16f;
            }
            c.baseValid = false;
        }

        static int Query(EntityBehaviorSelectionBoxes bh, BoxCache c, AttachmentPointAndPose[] arr,
            double ox, double oy, double oz, double dx, double dy, double dz, out double hx, out double hy, out double hz)
        {
            hx = hy = hz = 0;
            bool rebuiltAll = false;
            if (c.arr != arr || c.n != arr.Length) { EnsureArrays(c, arr); rebuiltAll = true; }
            int n = c.n;
            var entity = bh.entity;

            // ---- entity-level transform inputs (exactly what vanilla applyBoxTransform reads) ----
            var esr = entity.Properties.Client.Renderer as EntityShapeRenderer;
            float yaw = entity.Pos.Yaw;
            float y2 = entity.SelectionBox.Y2;
            float size = entity.Properties.Client.Size;
            float xa = 0, ya = 0, za = 0, sw = 0;
            if (esr != null) { xa = esr.xangle; ya = esr.yangle; za = esr.zangle; sw = esr.nowSwivelRad; }

            bool baseDirty = rebuiltAll || !c.baseValid
                || c.kYaw != Bits(yaw) || c.kY2 != Bits(y2) || c.kSize != Bits(size)
                || !ReferenceEquals(c.kEsr, esr)
                || c.kXa != Bits(xa) || c.kYa != Bits(ya) || c.kZa != Bits(za) || c.kSw != Bits(sw);

            float[] B = null;
            bool anyDirty = false;
            for (int i = 0; i < n; i++)
            {
                float[] anim = arr[i].AnimModelMatrix;
                var key = new Span<float>(c.animKey, i * 16, 16);
                bool dirty = baseDirty || !MemoryMarshal.AsBytes(new ReadOnlySpan<float>(anim, 0, 16)).SequenceEqual(MemoryMarshal.AsBytes((ReadOnlySpan<float>)key));
                if (!dirty) continue;
                if (!anyDirty)
                {
                    anyDirty = true;
                    B = BuildBase(esr, yaw, y2, size);
                }
                new ReadOnlySpan<float>(anim, 0, 16).CopyTo(key);
                RebuildBox(c, i, B, anim);
            }

            if (anyDirty)
            {
                if (Stats) CacheMisses++;
                c.kYaw = Bits(yaw); c.kY2 = Bits(y2); c.kSize = Bits(size); c.kEsr = esr;
                c.kXa = Bits(xa); c.kYa = Bits(ya); c.kZa = Bits(za); c.kSw = Bits(sw);
                c.baseValid = true;
                RebuildEntitySphere(c);
            }
            else if (Stats) CacheHits++;

            // ---- ray constants (vanilla: rayLengthSq = Length*Length with Length = dir.Length()) ----
            double len = Math.Sqrt(dx * dx + dy * dy + dz * dz);
            double rayLengthSq = len * len;
            double dd = dx * dx + dy * dy + dz * dz;

            if (EarlyOut && c.entOk && dd > 0 && SphereRejects(c.ecx, c.ecy, c.ecz, c.er, ox, oy, oz, dx, dy, dz, dd, len))
            {
                if (Stats) EntityRejects++;
                return -1;
            }

            int foundIndex = -1;
            double foundDistance = double.MaxValue;
            float[] M = c.m, I = c.inv;

            for (int i = 0; i < n; i++)
            {
                int b4 = i * 4;
                if (EarlyOut && c.sphOk[i] && dd > 0 && SphereRejects(c.sph[b4], c.sph[b4 + 1], c.sph[b4 + 2], c.sph[b4 + 3], ox, oy, oz, dx, dy, dz, dd, len))
                {
                    if (Stats) BoxSkips++;
                    continue;
                }
                if (Stats) BoxTests++;
                int o = i * 16;
                if (!c.invOk[i])
                {
                    // vanilla: mvMatInv.Set(mvmat.Values).Invert();  (Invert leaves the copy untouched if det==0)
                    Array.Copy(M, o, I, o, 16);
                    InvertAt(I, o);
                    c.invOk[i] = true;
                }

                // vanilla: obbSpaceOrigin = mvMatInv.TransformVector(rayOrigin w=1); obbSpaceDirection (w=0)
                Xf(I, o, ox, oy, oz, 1.0, out double rox, out double roy, out double roz);
                Xf(I, o, dx, dy, dz, 0.0, out double rdx, out double rdy, out double rdz);

                if (TestIntersectionStd(rox, roy, roz, rdx, rdy, rdz, out double px, out double py, out double pz))
                {
                    Xf(M, o, px, py, pz, 1.0, out double tx, out double ty, out double tz);
                    double ex = tx - ox, ey = ty - oy, ez = tz - oz;
                    double dist = ex * ex + ey * ey + ez * ez;
                    if (foundIndex >= 0 && foundDistance < dist || rayLengthSq < dist) continue;
                    hx = tx; hy = ty; hz = tz;
                    foundDistance = dist;
                    foundIndex = i;
                }
            }
            return foundIndex;
        }

        static float[] BuildBase(EntityShapeRenderer esr, float yaw, float y2, float size)
        {
            var B = tBase ??= new float[16];
            Mat4f.Identity(B);
            Mat4f.RotateY(B, B, (float)Math.PI / 2f + yaw);
            if (esr != null)
            {
                Mat4f.Translate(B, B, 0f, y2 / 2f, 0f);
                Mat4f.RotateX(B, B, esr.xangle);
                Mat4f.RotateY(B, B, esr.yangle);
                Mat4f.RotateZ(B, B, esr.zangle);
                Mat4f.Translate(B, B, 0f, (0f - y2) / 2f, 0f);
            }
            Mat4f.Translate(B, B, 0f, 0.7f, 0f);
            Mat4f.RotateX(B, B, esr?.nowSwivelRad ?? 0f);
            Mat4f.Translate(B, B, 0f, -0.7f, 0f);
            Mat4f.Scale(B, B, size, size, size);
            Mat4f.Translate(B, B, -0.5f, 0f, -0.5f);
            return B;
        }

        [ThreadStatic] static float[] tM;

        static void RebuildBox(BoxCache c, int i, float[] B, float[] anim)
        {
            // Must operate on a 16-length array starting at 0 because Mat4f works on whole arrays.
            var m = tM ??= new float[16];
            Array.Copy(B, m, 16);
            Mat4f.Mul(m, m, anim);
            Mat4f.Scale(m, m, c.sizes[i * 3], c.sizes[i * 3 + 1], c.sizes[i * 3 + 2]);
            Array.Copy(m, 0, c.m, i * 16, 16);
            c.invOk[i] = false;

            // bounding sphere of the parallelepiped M*[0,1]^3 + conditioning-based margin
            double c0x = m[0], c0y = m[1], c0z = m[2];
            double c1x = m[4], c1y = m[5], c1z = m[6];
            double c2x = m[8], c2y = m[9], c2z = m[10];
            bool affine = m[3] == 0f && m[7] == 0f && m[11] == 0f && m[15] == 1f;
            double cx = 0.5 * (c0x + c1x + c2x) + m[12];
            double cy = 0.5 * (c0y + c1y + c2y) + m[13];
            double cz = 0.5 * (c0z + c1z + c2z) + m[14];
            double l0 = Math.Sqrt(c0x * c0x + c0y * c0y + c0z * c0z);
            double l1 = Math.Sqrt(c1x * c1x + c1y * c1y + c1z * c1z);
            double l2 = Math.Sqrt(c2x * c2x + c2y * c2y + c2z * c2z);
            double r = 0.5 * (l0 + l1 + l2);
            double det = c0x * (c1y * c2z - c1z * c2y) - c0y * (c1x * c2z - c1z * c2x) + c0z * (c1x * c2y - c1y * c2x);
            double fro2 = l0 * l0 + l1 * l1 + l2 * l2;
            double cond = Math.Abs(det) > 0 ? Math.Pow(fro2, 1.5) / Math.Abs(det) : double.PositiveInfinity;
            double scale = Math.Sqrt(cx * cx + cy * cy + cz * cz) + r + 1.0;
            double margin = 1e-5 * cond * scale + 1e-6 * scale;
            bool ok = affine && cond <= CondLimit && !double.IsNaN(cond) && double.IsFinite(cx) && double.IsFinite(cy) && double.IsFinite(cz) && double.IsFinite(r) && double.IsFinite(margin);
            int b4 = i * 4;
            c.sph[b4] = cx; c.sph[b4 + 1] = cy; c.sph[b4 + 2] = cz; c.sph[b4 + 3] = r + margin;
            c.sphOk[i] = ok;
        }

        static void RebuildEntitySphere(BoxCache c)
        {
            int n = c.n;
            bool ok = n > 0;
            double sx = 0, sy = 0, sz = 0;
            for (int i = 0; i < n; i++)
            {
                if (!c.sphOk[i]) { ok = false; break; }
                sx += c.sph[i * 4]; sy += c.sph[i * 4 + 1]; sz += c.sph[i * 4 + 2];
            }
            if (!ok) { c.entOk = false; return; }
            double ex = sx / n, ey = sy / n, ez = sz / n, er = 0;
            for (int i = 0; i < n; i++)
            {
                double ddx = c.sph[i * 4] - ex, ddy = c.sph[i * 4 + 1] - ey, ddz = c.sph[i * 4 + 2] - ez;
                double rr = Math.Sqrt(ddx * ddx + ddy * ddy + ddz * ddz) + c.sph[i * 4 + 3];
                if (rr > er) er = rr;
            }
            er = er * (1 + 1e-9) + 1e-9;
            c.ecx = ex; c.ecy = ey; c.ecz = ez; c.er = er; c.entOk = double.IsFinite(er);
        }

        /// <summary>True if the half-line o + t*d (t>=0) provably misses the sphere, or every point of the
        /// sphere is farther than len from o (vanilla rejects hits with dist^2 > rayLengthSq).</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static bool SphereRejects(double cx, double cy, double cz, double r, double ox, double oy, double oz, double dx, double dy, double dz, double dd, double len)
        {
            double ocx = cx - ox, ocy = cy - oy, ocz = cz - oz;
            double oc2 = ocx * ocx + ocy * ocy + ocz * ocz;
            double lim = len + r;
            if (oc2 > lim * lim * (1 + 1e-9) + 1e-9) return true; // whole sphere beyond ray length
            double t = (ocx * dx + ocy * dy + ocz * dz) / dd;
            if (t < 0) t = 0;
            double qx = ox + dx * t - cx, qy = oy + dy * t - cy, qz = oz + dz * t - cz;
            double q2 = qx * qx + qy * qy + qz * qz;
            return q2 > r * r * (1 + 1e-9) + 1e-9;
        }

        /// <summary>Exactly Mat4f.MulWithVec4(float[], Vec4d, Vec4d), reading the matrix at offset o.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static void Xf(float[] m, int o, double x, double y, double z, double w, out double rx, out double ry, out double rz)
        {
            double a = 0.0;
            a += (double)m[o + 0] * x; a += (double)m[o + 4] * y; a += (double)m[o + 8] * z; a += (double)m[o + 12] * w;
            rx = a;
            a = 0.0;
            a += (double)m[o + 1] * x; a += (double)m[o + 5] * y; a += (double)m[o + 9] * z; a += (double)m[o + 13] * w;
            ry = a;
            a = 0.0;
            a += (double)m[o + 2] * x; a += (double)m[o + 6] * y; a += (double)m[o + 10] * z; a += (double)m[o + 14] * w;
            rz = a;
        }

        [ThreadStatic] static float[] tInv;
        static void InvertAt(float[] arr, int o)
        {
            var t = tInv ??= new float[16];
            Array.Copy(arr, o, t, 0, 16);
            Mat4f.Invert(t, t); // returns null and leaves t untouched when det == 0, same as vanilla
            Array.Copy(t, 0, arr, o, 16);
        }

        /// <summary>Exactly vanilla Testintersection(standardbox(0,0,0,1,1,1), ray) without allocations.</summary>
        static bool TestIntersectionStd(double ox, double oy, double oz, double dx, double dy, double dz, out double hx, out double hy, out double hz)
        {
            const double X1 = 0.0, Y1 = 0.0, Z1 = 0.0, X2 = 1.0, Y2 = 1.0, Z2 = 1.0;
            double w = X2 - X1, h = Y2 - Y1, l = Z2 - Z1;
            var faces = BlockFacing.ALLFACES;
            for (int i = 0; i < 6; i++)
            {
                BlockFacing f = faces[i];
                Vec3i n = f.Normali;
                double demon = (double)n.X * dx + (double)n.Y * dy + (double)n.Z * dz;
                if (!(demon < -1E-05)) continue;
                Vec3f pc = f.PlaneCenter;
                double pcx = pc.X; pcx *= w; pcx += X1;
                double pcy = pc.Y; pcy *= h; pcy += Y1;
                double pcz = pc.Z; pcz *= l; pcz += Z1;
                double ptx = pcx - ox, pty = pcy - oy, ptz = pcz - oz;
                double t = (ptx * (double)n.X + pty * (double)n.Y + ptz * (double)n.Z) / demon;
                if (t >= 0.0)
                {
                    double qx = ox + dx * t, qy = oy + dy * t, qz = oz + dz * t;
                    double ex = qx - pcx, ey = qy - pcy, ez = qz - pcz;
                    if (Math.Abs(ex) <= w / 2.0 && Math.Abs(ey) <= h / 2.0 && Math.Abs(ez) <= l / 2.0)
                    {
                        hx = qx; hy = qy; hz = qz;
                        return true;
                    }
                }
            }
            hx = hy = hz = 0;
            return false;
        }

        // ---------- helpers that mirror the vanilla wrappers ----------

        /// <summary>Mirror of vanilla getHitIndex() (client, player's own picking ray), allocation-free.</summary>
        public static int GetHitIndexFromPlayer(EntityBehaviorSelectionBoxes bh, ICoreClientAPI capi)
        {
            var eplr = capi.World.Player.Entity;
            var ep = eplr.Pos;
            var np = bh.entity.Pos;
            var eye = eplr.LocalEyePos;
            // Vec3d (eplr.Pos.XYZ + LocalEyePos) - entity.Pos.XYZ
            double px = (ep.X + eye.X) - np.X;
            double py = (ep.InternalY + eye.Y) - np.InternalY;
            double pz = (ep.Z + eye.Z) - np.Z;
            float pitch = ep.Pitch, yaw = ep.Yaw;
            double offset = capi.World.Player.WorldData.PickingRange;
            // Vec3d.AheadCopy(double, float, float) then SubCopy(pos)
            float cp = GameMath.Cos(pitch);
            float sp = GameMath.Sin(pitch);
            float cy = GameMath.Cos(yaw);
            float sy = GameMath.Sin(yaw);
            double ax = px - (double)(cp * sy) * offset;
            double ay = py + (double)sp * offset;
            double az = pz - (double)(cp * cy) * offset;
            return GetHitIndex(bh, px, py, pz, ax - px, ay - py, az - pz, out _, out _, out _);
        }
    }
}