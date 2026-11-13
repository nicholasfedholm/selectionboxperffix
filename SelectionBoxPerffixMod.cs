using System;
using HarmonyLib;
using Vintagestory.API.Client;
using Vintagestory.API.Common;
using Vintagestory.API.MathTools;
using Vintagestory.GameContent;

namespace SelectionBoxPerffix
{
    public class SelectionBoxPerffixConfig
    {
        /// <summary>Master switch. false = mod does nothing (pure vanilla).</summary>
        public bool Enabled = true;
        /// <summary>Per-entity cache of box matrices/inverses, keyed bit-exactly on all inputs.</summary>
        public bool UseCache = true;
        /// <summary>Conservative bounding-sphere rejects (entity and per box).</summary>
        public bool EarlyOut = true;
        /// <summary>Run BOTH vanilla and perffix on every getHitIndex(Ray) call and count mismatches (slow, for in-game validation only).</summary>
        public bool VerifyMode = false;
        /// <summary>Log call/skip/cache counters every 60 s.</summary>
        public bool LogStats = false;
    }

    public class SelectionBoxPerffixMod : ModSystem
    {
        public const string HarmonyId = "nick.selectionboxperffix";
        static Harmony harmony;
        static int patchedSides;
        internal static SelectionBoxPerffixConfig Config;
        internal static ILogger Log;
        public static long VerifyCalls, VerifyMismatches;

        public override double ExecuteOrder() => 0.0;

        public override void StartPre(ICoreAPI api)
        {
            if (Config == null)
            {
                try { Config = api.LoadModConfig<SelectionBoxPerffixConfig>("SelectionBoxPerffix.json"); } catch (Exception e) { api.Logger.Warning("[selectionboxperffix] bad config, using defaults: {0}", e.Message); }
                if (Config == null) { Config = new SelectionBoxPerffixConfig(); try { api.StoreModConfig(Config, "SelectionBoxPerffix.json"); } catch { } }
            }
            Log = api.Logger;
            SelBoxCore.UseCache = Config.UseCache;
            SelBoxCore.EarlyOut = Config.EarlyOut;
            SelBoxCore.Stats = Config.LogStats;

            // Singleplayer: client and server share one process + one Harmony; patch once.
            if (harmony == null && Config.Enabled)
            {
                harmony = new Harmony(HarmonyId);
                SelBoxPatches.Apply(harmony, Config.VerifyMode);
                api.Logger.Notification("[selectionboxperffix] patched EntityBehaviorSelectionBoxes (cache={0}, earlyOut={1}, verify={2})", Config.UseCache, Config.EarlyOut, Config.VerifyMode);
            }
            patchedSides++;
        }

        public override void Start(ICoreAPI api)
        {
            if (Config.LogStats || Config.VerifyMode)
            {
                api.Event.RegisterGameTickListener(_ =>
                {
                    api.Logger.Notification("[selectionboxperffix] {0} calls={1} entityRejects={2} boxTests={3} boxSkips={4} cacheHits={5} cacheMisses={6} verifyCalls={7} verifyMismatches={8}",
                        api.Side, SelBoxCore.Calls, SelBoxCore.EntityRejects, SelBoxCore.BoxTests, SelBoxCore.BoxSkips, SelBoxCore.CacheHits, SelBoxCore.CacheMisses, VerifyCalls, VerifyMismatches);
                }, 60000);
            }
        }

        public override void Dispose()
        {
            if (--patchedSides <= 0 && harmony != null)
            {
                harmony.UnpatchAll(HarmonyId);
                harmony = null;
                Config = null;
            }
        }
    }

    public static class SelBoxPatches
    {
        public static void Apply(Harmony h, bool verify)
        {
            var t = typeof(EntityBehaviorSelectionBoxes);
            var mRay = AccessTools.Method(t, "getHitIndex", new[] { typeof(Ray) }) ?? throw new MissingMethodException(t.FullName, "getHitIndex(Ray)");
            if (verify)
            {
                h.Patch(mRay, prefix: new HarmonyMethod(typeof(SelBoxPatches), nameof(VerifyPrefix)), postfix: new HarmonyMethod(typeof(SelBoxPatches), nameof(VerifyPostfix)));
                return;
            }
            var mNoArg = AccessTools.Method(t, "getHitIndex", Type.EmptyTypes) ?? throw new MissingMethodException(t.FullName, "getHitIndex()");
            var mIntersects = AccessTools.Method(t, nameof(EntityBehaviorSelectionBoxes.IntersectsRay)) ?? throw new MissingMethodException(t.FullName, "IntersectsRay");
            h.Patch(mRay, prefix: new HarmonyMethod(typeof(SelBoxPatches), nameof(GetHitIndexRayPrefix)));
            h.Patch(mNoArg, prefix: new HarmonyMethod(typeof(SelBoxPatches), nameof(GetHitIndexNoArgPrefix)));
            h.Patch(mIntersects, prefix: new HarmonyMethod(typeof(SelBoxPatches), nameof(IntersectsRayPrefix)));
        }

        // private int getHitIndex(Ray pickingray)
        public static bool GetHitIndexRayPrefix(EntityBehaviorSelectionBoxes __instance, Ray pickingray, ref int __result)
        {
            Vec3d o = pickingray.origin, d = pickingray.dir;
            __result = SelBoxCore.GetHitIndex(__instance, o.X, o.Y, o.Z, d.X, d.Y, d.Z, out _, out _, out _);
            return false;
        }

        // private int getHitIndex()
        public static bool GetHitIndexNoArgPrefix(EntityBehaviorSelectionBoxes __instance, ref int __result)
        {
            if (!(__instance.entity.Api is ICoreClientAPI capi)) return true; // vanilla would NRE too; let it
            __result = SelBoxCore.GetHitIndexFromPlayer(__instance, capi);
            return false;
        }

        // public override bool IntersectsRay(Ray ray, AABBIntersectionTest intersectionTester, out double intersectionDistance, ref int selectionBoxIndex, ref EnumHandling handled)
        public static bool IntersectsRayPrefix(EntityBehaviorSelectionBoxes __instance, Ray ray, AABBIntersectionTest intersectionTester,
            ref double intersectionDistance, ref int selectionBoxIndex, ref EnumHandling handled, ref bool __result)
        {
            var pos = __instance.entity.Pos;
            double px = pos.X, py = pos.InternalY, pz = pos.Z;
            Vec3d ro = ray.origin, rd = ray.dir;
            // vanilla: new Ray(ray.origin - entity.Pos.XYZ, ray.dir)
            int index = SelBoxCore.GetHitIndex(__instance, ro.X - px, ro.Y - py, ro.Z - pz, rd.X, rd.Y, rd.Z, out double hx, out double hy, out double hz);
            if (index >= 0)
            {
                intersectionDistance = Math.Sqrt(hx * hx + hy * hy + hz * hz);           // hitPositionAABBSpace.Length()
                intersectionTester.hitPosition = new Vec3d(hx + px, hy + py, hz + pz);     // hitPositionAABBSpace.AddCopy(entity.Pos.XYZ) (stored by caller -> must be a fresh object)
                selectionBoxIndex = 1 + index;
                handled = EnumHandling.PreventDefault;
                __result = true;
                return false;
            }
            intersectionDistance = double.MaxValue;
            __result = false;
            return false;
        }

        // ---- verify mode: compute ours, let vanilla run, compare ----
        [ThreadStatic] static int vIdx;
        public static void VerifyPrefix(EntityBehaviorSelectionBoxes __instance, Ray pickingray)
        {
            Vec3d o = pickingray.origin, d = pickingray.dir;
            vIdx = SelBoxCore.GetHitIndex(__instance, o.X, o.Y, o.Z, d.X, d.Y, d.Z, out _, out _, out _);
        }
        public static void VerifyPostfix(EntityBehaviorSelectionBoxes __instance, int __result)
        {
            System.Threading.Interlocked.Increment(ref SelectionBoxPerffixMod.VerifyCalls);
            if (__result != vIdx)
            {
                long n = System.Threading.Interlocked.Increment(ref SelectionBoxPerffixMod.VerifyMismatches);
                if (n <= 20) SelectionBoxPerffixMod.Log?.Warning("[selectionboxperffix] VERIFY MISMATCH on {0}: vanilla={1} perffix={2}", __instance.entity?.Code, __result, vIdx);
            }
        }
    }
}