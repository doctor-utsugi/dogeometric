using Dogeometric.Core.Geometry;
using Dogeometric.Core.Modeling;

namespace Dogeometric.Core.IO;

/// <summary>Repair Mesh's options.</summary>
public sealed record MeshRepairOptions
{
    /// <summary>Open borders closer than this (mm) are sewn together.</summary>
    public double GapTolerance { get; init; } = 0.05;

    public bool FillHoles { get; init; } = true;

    /// <summary>Pieces whose bounding box is smaller than this share of the largest piece's are erased (0 keeps all).</summary>
    public double LoosePieceFraction { get; init; } = 0.01;

    /// <summary>Erases closed shells lying inside others (hollow parts become filled).</summary>
    public bool RemoveCavities { get; init; }

    /// <summary>Overlapping closed shells become one solid (done by Solid Tools, which has the boolean engine).</summary>
    public bool MergeShells { get; init; } = true;
}

/// <summary>What Repair Mesh did, and what it could not fix.</summary>
public sealed class MeshRepairReport
{
    public int Degenerate { get; set; }
    public int Duplicates { get; set; }
    public int InternalWalls { get; set; }
    public int Stitched { get; set; }
    public int TJunctions { get; set; }
    public int Flipped { get; set; }
    public int HolesFilled { get; set; }
    public int LoosePieces { get; set; }
    public int Cavities { get; set; }
    public int ShellsMerged { get; set; }
    public int Slivers { get; set; }

    /// <summary>Set when Solid Tools could not merge the shells (they stay as they were).</summary>
    public string? MergeError { get; set; }

    /// <summary>Shells still open after the repair; while there are any, the shells are not merged.</summary>
    public int OpenShells { get; set; }

    public int BoundaryEdges { get; set; }
    public int NonManifoldEdges { get; set; }
    public bool IsClosed => BoundaryEdges == 0 && NonManifoldEdges == 0;

    public IEnumerable<(string Label, int Count)> Lines()
    {
        yield return ("Degenerate triangles erased", Degenerate);
        yield return ("Duplicate triangles erased", Duplicates);
        yield return ("Internal walls erased", InternalWalls);
        yield return ("Vertices stitched", Stitched);
        yield return ("T-junctions closed", TJunctions);
        yield return ("Triangles flipped", Flipped);
        yield return ("Holes filled", HolesFilled);
        yield return ("Loose pieces erased", LoosePieces);
        yield return ("Cavities erased", Cavities);
        yield return ("Shells merged", ShellsMerged);
        yield return ("Slivers removed", Slivers);
    }
}

/// <summary>
/// Repair Mesh: turns an imported triangle soup (STL, OBJ…) into closed shells — erases degenerate and duplicate
/// triangles, sews open borders closer than a tolerance (vertex to vertex and vertex to edge), orients every shell
/// outwards (cavities inwards), fills the holes left and erases loose bits.
/// </summary>
public static class MeshRepair
{
    /// <param name="Bodies">Triangle numbers of each body: a closed outer shell with its cavities, or an open shell.</param>
    public sealed record Result(List<Vec3> Positions, List<int> Triangles, List<Material?> Materials, List<List<int>> Bodies, MeshRepairReport Report);

    private record struct Tri(int A, int B, int C)
    {
        public bool Has(int v) => A == v || B == v || C == v;

        public IEnumerable<(int, int)> Directed() => [(A, B), (B, C), (C, A)];

        public Tri Flipped => new(A, C, B);
    }

    private sealed class Mesh
    {
        public readonly List<Vec3> P = [];
        public List<Tri> T = [];
        public List<Material?> M = [];

        public static (int, int) Key(int a, int b) => a < b ? (a, b) : (b, a);

        /// <summary>The triangles using each undirected edge.</summary>
        public Dictionary<(int, int), List<int>> Edges()
        {
            var edges = new Dictionary<(int, int), List<int>>();
            for (var i = 0; i < T.Count; i++)
            {
                foreach (var (a, b) in T[i].Directed())
                {
                    var key = Key(a, b);
                    if (!edges.TryGetValue(key, out var list))
                        edges[key] = list = [];
                    list.Add(i);
                }
            }
            return edges;
        }

        public void Keep(Func<int, bool> keep)
        {
            var t = new List<Tri>();
            var m = new List<Material?>();
            for (var i = 0; i < T.Count; i++)
            {
                if (!keep(i))
                    continue;
                t.Add(T[i]);
                m.Add(M[i]);
            }
            (T, M) = (t, m);
        }

        public void Remap(int[] map)
        {
            for (var i = 0; i < T.Count; i++)
                T[i] = new Tri(map[T[i].A], map[T[i].B], map[T[i].C]);
        }

        public Vec3 Normal(Tri t) => (P[t.B] - P[t.A]).Cross(P[t.C] - P[t.A]);

        /// <summary>
        /// The triangles that continue each other's surface across each edge. Two at an edge pair up. Where more
        /// meet (parts touching along an edge), each pairs with the one of opposite winding from the same part,
        /// as far as the pairs already found tell parts apart; the rest pair with their neighbour around the edge
        /// on the side of their material.
        /// </summary>
        public List<(int, int)> Pairs()
        {
            var parent = Enumerable.Range(0, T.Count).ToArray();
            int Root(int i)
            {
                while (parent[i] != i)
                    i = parent[i] = parent[parent[i]];
                return i;
            }
            var pairs = new List<(int, int)>();
            var crowded = new List<((int, int) Edge, List<int> Tris)>();
            foreach (var (edge, tris) in Edges())
            {
                if (tris.Count == 2)
                {
                    pairs.Add((tris[0], tris[1]));
                    parent[Root(tris[1])] = Root(tris[0]);
                }
                else if (tris.Count >= 4 && tris.Count % 2 == 0)
                    crowded.Add((edge, tris));
            }

            for (var progress = true; progress;)
            {
                progress = false;
                foreach (var c in crowded.ToList())
                {
                    var (a, b) = c.Edge;
                    var parts = c.Tris.GroupBy(Root).ToList();
                    if (parts.Any(g => g.Count() != 2 || g.Count(t => T[t].Directed().Contains((a, b))) != 1))
                        continue;
                    foreach (var g in parts)
                    {
                        pairs.Add((g.First(), g.Last()));
                        parent[Root(g.Last())] = Root(g.First());
                    }
                    crowded.Remove(c);
                    progress = true;
                }
            }

            foreach (var ((a, b), tris) in crowded)
            {
                var d = (P[b] - P[a]).Normalized();
                var (u, v) = Polygon.PlaneAxes(d);
                // Around a → b: a triangle running a → b faces towards growing angle, so its material lies on the
                // side of the one before it, which must run b → a.
                var around = tris.Select(t =>
                {
                    var c = T[t].A != a && T[t].A != b ? T[t].A : T[t].B != a && T[t].B != b ? T[t].B : T[t].C;
                    var w = P[c] - P[a];
                    return (T: t, Angle: Math.Atan2(w.Dot(v), w.Dot(u)), Forward: T[t].Directed().Contains((a, b)));
                }).OrderBy(x => x.Angle).ToList();
                // Two parts pressed face to face meet at one angle; the faces in contact are never partners, so
                // the one running a → b goes first.
                for (var i = 1; i < around.Count; i++)
                    if (around[i].Angle - around[i - 1].Angle < 1e-4 && around[i].Forward && !around[i - 1].Forward)
                        (around[i], around[i - 1]) = (around[i - 1], around[i]);
                var found = new List<(int, int)>();
                for (var i = 0; i < around.Count; i++)
                {
                    if (!around[i].Forward)
                        continue;
                    var before = around[(i - 1 + around.Count) % around.Count];
                    if (before.Forward)
                        break;
                    found.Add((around[i].T, before.T));
                }
                if (found.Count * 2 == tris.Count)
                    pairs.AddRange(found);
            }
            return pairs;
        }

        /// <summary>Triangles connected through <see cref="Pairs"/>: parts touching along an edge stay apart.</summary>
        public List<List<int>> Components()
        {
            var parent = Enumerable.Range(0, T.Count).ToArray();
            int Root(int i)
            {
                while (parent[i] != i)
                    i = parent[i] = parent[parent[i]];
                return i;
            }
            foreach (var (x, y) in Pairs())
                parent[Root(y)] = Root(x);
            return Enumerable.Range(0, T.Count).GroupBy(Root).Select(g => g.ToList()).ToList();
        }

        /// <summary>Every edge of the triangles is used by exactly two of them.</summary>
        public bool IsClosed(IEnumerable<int> tris)
        {
            var uses = new Dictionary<(int, int), int>();
            foreach (var t in tris)
                foreach (var (a, b) in T[t].Directed())
                    uses[Key(a, b)] = uses.GetValueOrDefault(Key(a, b)) + 1;
            return uses.Values.All(n => n == 2);
        }
    }

    public static Result Run(IReadOnlyList<Triangle> input, MeshRepairOptions? options = null)
    {
        var o = options ?? new MeshRepairOptions();
        var report = new MeshRepairReport();
        var m = Index(input);
        Clean(m, report);
        var tolerance = Math.Max(o.GapTolerance, Tolerance.Length);
        report.Stitched = Stitch(m, tolerance);
        Clean(m, report);
        for (var pass = 0; pass < 8; pass++)
        {
            var split = SplitTJunctions(m, tolerance);
            if (split == 0)
                break;
            report.TJunctions += split;
        }
        Clean(m, report);
        report.InternalWalls += RemoveInternalFaces(m);
        report.Flipped += Orient(m);
        report.LoosePieces = RemoveLoosePieces(m, o.LoosePieceFraction);
        if (o.FillHoles)
            report.HolesFilled = FillHoles(m);
        var bodies = Shells(m, o, report);
        CountOpenEdges(m, report);
        return Compact(m, bodies, report);
    }

    /// <summary>
    /// Removes slivers from a closed triangle mesh (as booleans leave where surfaces nearly touch): edges shorter
    /// than <paramref name="length"/> collapse to their middle, and triangles flatter than it get their long edge
    /// split under the far corner, which then collapses onto the split. Collapses that would pinch the surface or
    /// turn a triangle over are skipped. Returns the slivers removed.
    /// </summary>
    public static int RemoveSlivers(List<Vec3> positions, List<int> triangles, double length = Tolerance.Length)
    {
        var t = new List<Tri>(triangles.Count / 3);
        for (var i = 0; i + 2 < triangles.Count; i += 3)
            t.Add(new Tri(triangles[i], triangles[i + 1], triangles[i + 2]));
        var alive = Enumerable.Repeat(true, t.Count).ToList();
        var around = positions.Select(_ => new HashSet<int>()).ToList();
        for (var i = 0; i < t.Count; i++)
            foreach (var v in new[] { t[i].A, t[i].B, t[i].C })
                around[v].Add(i);

        Vec3 Normal(Tri x) => (positions[x.B] - positions[x.A]).Cross(positions[x.C] - positions[x.A]);
        int Third(Tri x, int a, int b) => x.A != a && x.A != b ? x.A : x.B != a && x.B != b ? x.B : x.C;
        Tri Swap(Tri x, int from, int to) => new(x.A == from ? to : x.A, x.B == from ? to : x.B, x.C == from ? to : x.C);
        HashSet<int> Ring(int v) => around[v].SelectMany(i => new[] { t[i].A, t[i].B, t[i].C }).Where(x => x != v).ToHashSet();
        void Kill(int i)
        {
            alive[i] = false;
            foreach (var v in new[] { t[i].A, t[i].B, t[i].C })
                around[v].Remove(i);
        }

        bool Collapse(int a, int b)
        {
            var shared = around[a].Intersect(around[b]).ToList();
            if (shared.Count != 2)
                return false;
            var opposite = shared.Select(i => Third(t[i], a, b)).ToHashSet();
            var common = Ring(a);
            common.IntersectWith(Ring(b));
            if (!common.SetEquals(opposite))
                return false;
            // To the middle, or else to either end, wherever no neighbouring triangle turns over.
            var target = new[] { (positions[a] + positions[b]) / 2, positions[a], positions[b] }.Cast<Vec3?>()
                .FirstOrDefault(to => around[a].Concat(around[b]).Except(shared).All(i =>
                {
                    var moved = Swap(t[i], b, a);
                    Vec3 Pos(int v) => v == a ? to!.Value : positions[v];
                    return Normal(t[i]).Dot((Pos(moved.B) - Pos(moved.A)).Cross(Pos(moved.C) - Pos(moved.A))) > 0;
                }));
            if (target is not { } to)
                return false;
            positions[a] = to;
            foreach (var i in shared)
                Kill(i);
            foreach (var i in around[b].ToList())
            {
                t[i] = Swap(t[i], b, a);
                around[a].Add(i);
            }
            around[b].Clear();
            return true;
        }

        var removed = 0;
        for (var pass = 0; pass < 4; pass++)
        {
            var changed = 0;
            var count = t.Count;
            for (var i = 0; i < count; i++)
            {
                if (!alive[i])
                    continue;
                var x = t[i];
                var (shortest, a, b) = new[] { (positions[x.A].DistanceTo(positions[x.B]), x.A, x.B),
                    (positions[x.B].DistanceTo(positions[x.C]), x.B, x.C), (positions[x.C].DistanceTo(positions[x.A]), x.C, x.A) }.MinBy(e => e.Item1);
                if (shortest < length)
                {
                    if (Collapse(a, b))
                    {
                        removed++;
                        changed++;
                    }
                    continue;
                }
                // A cap: split the long edge under the far corner, so a short edge appears there.
                var (longest, la, lb) = new[] { (positions[x.A].DistanceTo(positions[x.B]), x.A, x.B),
                    (positions[x.B].DistanceTo(positions[x.C]), x.B, x.C), (positions[x.C].DistanceTo(positions[x.A]), x.C, x.A) }.MaxBy(e => e.Item1);
                if (Normal(x).Length / longest >= length)
                    continue;
                var c = Third(x, la, lb);
                var across = around[la].Intersect(around[lb]).Where(j => j != i).ToList();
                if (across.Count != 1)
                    continue;
                var ab = positions[lb] - positions[la];
                var s = (positions[c] - positions[la]).Dot(ab) / ab.LengthSquared;
                // Only when the corner lies over the edge: the split point must be clear of both ends.
                if (s * longest < length || (1 - s) * longest < length)
                    continue;
                var p = positions.Count;
                positions.Add(positions[la] + ab * s);
                around.Add([]);
                foreach (var j in new[] { i, across[0] })
                {
                    var y = t[j];
                    // Rotate so the split edge runs u → w, then cut it at p.
                    var (u, w, z) = (y.A, y.B, y.C);
                    while (!((u == la && w == lb) || (u == lb && w == la)))
                        (u, w, z) = (w, z, u);
                    Kill(j);
                    foreach (var n in new[] { new Tri(u, p, z), new Tri(p, w, z) })
                    {
                        t.Add(n);
                        alive.Add(true);
                        foreach (var v in new[] { n.A, n.B, n.C })
                            around[v].Add(t.Count - 1);
                    }
                }
                if (Collapse(c, p))
                {
                    removed++;
                    changed++;
                }
            }
            if (changed == 0)
                break;
        }

        triangles.Clear();
        for (var i = 0; i < t.Count; i++)
        {
            if (!alive[i])
                continue;
            triangles.Add(t[i].A);
            triangles.Add(t[i].B);
            triangles.Add(t[i].C);
        }
        return removed;
    }

    private static Mesh Index(IReadOnlyList<Triangle> input)
    {
        var m = new Mesh();
        var cell = Tolerance.Length * 2;
        var grid = new Dictionary<(long, long, long), List<int>>();
        int Id(Vec3 p)
        {
            var (x, y, z) = ((long)Math.Floor(p.X / cell), (long)Math.Floor(p.Y / cell), (long)Math.Floor(p.Z / cell));
            for (var dx = -1; dx <= 1; dx++)
                for (var dy = -1; dy <= 1; dy++)
                    for (var dz = -1; dz <= 1; dz++)
                        if (grid.TryGetValue((x + dx, y + dy, z + dz), out var list))
                            foreach (var i in list)
                                if (m.P[i].DistanceTo(p) <= Tolerance.Length)
                                    return i;
            if (!grid.TryGetValue((x, y, z), out var own))
                grid[(x, y, z)] = own = [];
            own.Add(m.P.Count);
            m.P.Add(p);
            return m.P.Count - 1;
        }
        foreach (var t in input)
        {
            m.T.Add(new Tri(Id(t.A), Id(t.B), Id(t.C)));
            m.M.Add(t.Material);
        }
        return m;
    }

    /// <summary>
    /// Erases triangles with repeated or collinear corners, and repeated triangles: one copy stays when they face
    /// the same way, none when they face each other (a zero-thickness wall inside the volume).
    /// </summary>
    private static void Clean(Mesh m, MeshRepairReport report)
    {
        var drop = new bool[m.T.Count];
        for (var i = 0; i < m.T.Count; i++)
        {
            var t = m.T[i];
            if (t.A == t.B || t.B == t.C || t.C == t.A)
            {
                drop[i] = true;
                report.Degenerate++;
                continue;
            }
            var longest = Math.Max((m.P[t.B] - m.P[t.A]).LengthSquared, Math.Max((m.P[t.C] - m.P[t.B]).LengthSquared, (m.P[t.A] - m.P[t.C]).LengthSquared));
            if (m.Normal(t).Length <= 1e-9 * longest)
            {
                drop[i] = true;
                report.Degenerate++;
            }
        }

        var bySet = new Dictionary<(int, int, int), List<int>>();
        for (var i = 0; i < m.T.Count; i++)
        {
            if (drop[i])
                continue;
            var t = m.T[i];
            int[] s = [t.A, t.B, t.C];
            Array.Sort(s);
            if (!bySet.TryGetValue((s[0], s[1], s[2]), out var list))
                bySet[(s[0], s[1], s[2])] = list = [];
            list.Add(i);
        }
        foreach (var list in bySet.Values.Where(l => l.Count > 1))
        {
            var first = m.T[list[0]];
            var same = list.Where(i => SameWinding(m.T[i], first)).ToList();
            var opposite = list.Except(same).ToList();
            var walls = Math.Min(same.Count, opposite.Count);
            // Facing pairs cancel; what is left over keeps one copy.
            var keep = same.Count > opposite.Count ? same[0] : opposite.Count > same.Count ? opposite[0] : -1;
            foreach (var i in list.Where(i => i != keep))
                drop[i] = true;
            report.InternalWalls += walls;
            report.Duplicates += list.Count - 2 * walls - (keep >= 0 ? 1 : 0);
        }
        m.Keep(i => !drop[i]);
    }

    private static bool SameWinding(Tri a, Tri b) =>
        (a.A, a.B, a.C) == (b.A, b.B, b.C) || (a.A, a.B, a.C) == (b.B, b.C, b.A) || (a.A, a.B, a.C) == (b.C, b.A, b.B);

    private static HashSet<int> BoundaryVertices(Dictionary<(int, int), List<int>> edges)
    {
        var set = new HashSet<int>();
        foreach (var ((a, b), tris) in edges)
        {
            if (tris.Count != 1)
                continue;
            set.Add(a);
            set.Add(b);
        }
        return set;
    }

    /// <summary>Merges vertices on open borders lying within <paramref name="tolerance"/> of each other.</summary>
    private static int Stitch(Mesh m, double tolerance)
    {
        var boundary = BoundaryVertices(m.Edges()).ToList();
        var grid = new Dictionary<(long, long, long), List<int>>();
        (long, long, long) Cell(Vec3 p) => ((long)Math.Floor(p.X / tolerance), (long)Math.Floor(p.Y / tolerance), (long)Math.Floor(p.Z / tolerance));
        foreach (var v in boundary)
        {
            var c = Cell(m.P[v]);
            if (!grid.TryGetValue(c, out var list))
                grid[c] = list = [];
            list.Add(v);
        }

        var map = Enumerable.Range(0, m.P.Count).ToArray();
        int Root(int i)
        {
            while (map[i] != i)
                i = map[i] = map[map[i]];
            return i;
        }
        var merged = 0;
        foreach (var v in boundary)
        {
            var (x, y, z) = Cell(m.P[v]);
            for (var dx = -1; dx <= 1; dx++)
                for (var dy = -1; dy <= 1; dy++)
                    for (var dz = -1; dz <= 1; dz++)
                        if (grid.TryGetValue((x + dx, y + dy, z + dz), out var list))
                            foreach (var w in list)
                            {
                                if (w <= v || m.P[v].DistanceTo(m.P[w]) > tolerance)
                                    continue;
                                var (rv, rw) = (Root(v), Root(w));
                                if (rv == rw)
                                    continue;
                                map[Math.Max(rv, rw)] = Math.Min(rv, rw);
                                merged++;
                            }
        }
        if (merged > 0)
            m.Remap(Enumerable.Range(0, m.P.Count).Select(Root).ToArray());
        return merged;
    }

    /// <summary>
    /// Border vertices lying on another border edge (within <paramref name="tolerance"/>) split that edge's
    /// triangle, so both sides of the seam share the vertex. One edge per triangle per pass.
    /// </summary>
    private static int SplitTJunctions(Mesh m, double tolerance)
    {
        var edges = m.Edges();
        var open = edges.Where(kv => kv.Value.Count == 1).Select(kv => (kv.Key.Item1, kv.Key.Item2, Tri: kv.Value[0])).ToList();
        if (open.Count == 0)
            return 0;
        var cell = Math.Max(tolerance * 4, open.Average(e => m.P[e.Item1].DistanceTo(m.P[e.Item2])));
        (long, long, long) Cell(Vec3 p) => ((long)Math.Floor(p.X / cell), (long)Math.Floor(p.Y / cell), (long)Math.Floor(p.Z / cell));
        var grid = new Dictionary<(long, long, long), List<int>>();
        for (var i = 0; i < open.Count; i++)
        {
            var (a, b) = (m.P[open[i].Item1], m.P[open[i].Item2]);
            var pad = new Vec3(tolerance, tolerance, tolerance);
            var lo = Cell(new Vec3(Math.Min(a.X, b.X), Math.Min(a.Y, b.Y), Math.Min(a.Z, b.Z)) - pad);
            var hi = Cell(new Vec3(Math.Max(a.X, b.X), Math.Max(a.Y, b.Y), Math.Max(a.Z, b.Z)) + pad);
            for (var x = lo.Item1; x <= hi.Item1; x++)
                for (var y = lo.Item2; y <= hi.Item2; y++)
                    for (var z = lo.Item3; z <= hi.Item3; z++)
                    {
                        if (!grid.TryGetValue((x, y, z), out var list))
                            grid[(x, y, z)] = list = [];
                        list.Add(i);
                    }
        }

        var splits = new Dictionary<int, List<(double T, int V)>>();
        foreach (var v in BoundaryVertices(edges))
        {
            var p = m.P[v];
            if (!grid.TryGetValue(Cell(p), out var candidates))
                continue;
            foreach (var i in candidates)
            {
                var (ia, ib, _) = open[i];
                if (ia == v || ib == v)
                    continue;
                var (a, b) = (m.P[ia], m.P[ib]);
                var ab = b - a;
                var len = ab.Length;
                var t = (p - a).Dot(ab) / (len * len);
                if (t * len <= tolerance || (1 - t) * len <= tolerance || p.DistanceTo(a + ab * t) > tolerance)
                    continue;
                if (!splits.TryGetValue(i, out var list))
                    splits[i] = list = [];
                if (list.All(s => s.V != v))
                    list.Add((t, v));
            }
        }

        var done = new HashSet<int>();
        var count = 0;
        foreach (var (i, list) in splits)
        {
            var (ia, ib, tri) = open[i];
            if (!done.Add(tri))
                continue;
            var t = m.T[tri];
            // Rotate the triangle so the split edge runs x → y in its own winding.
            var (x, y, z) = (t.A, t.B, t.C);
            while (Mesh.Key(x, y) != Mesh.Key(ia, ib))
                (x, y, z) = (y, z, x);
            var chain = list.OrderBy(s => s.T).Select(s => s.V).ToList();
            if (x != ia)
                chain.Reverse();
            var material = m.M[tri];
            var prev = x;
            m.T[tri] = new Tri(x, chain[0], z);
            for (var k = 0; k < chain.Count; k++)
            {
                var next = k + 1 < chain.Count ? chain[k + 1] : y;
                m.T.Add(new Tri(chain[k], next, z));
                m.M.Add(material);
            }
            count += chain.Count;
        }
        return count;
    }

    /// <summary>
    /// Erases internal faces: open patches whose whole border is edges shared by three or more triangles (a wall
    /// splitting a volume, a face left between two touching parts) and that lie inside the rest of the mesh.
    /// </summary>
    private static int RemoveInternalFaces(Mesh m)
    {
        var edges = m.Edges();
        if (edges.Values.All(l => l.Count <= 2))
            return 0;
        var parent = Enumerable.Range(0, m.T.Count).ToArray();
        int Root(int i)
        {
            while (parent[i] != i)
                i = parent[i] = parent[parent[i]];
            return i;
        }
        foreach (var (x, y) in m.Pairs())
            parent[Root(y)] = Root(x);
        var patches = Enumerable.Range(0, m.T.Count).GroupBy(Root).Select(g => g.ToList())
            .Where(p => !m.IsClosed(p) && p.All(t => m.T[t].Directed().All(e => edges[Mesh.Key(e.Item1, e.Item2)].Count != 1))
                && p.Any(t => m.T[t].Directed().Any(e => edges[Mesh.Key(e.Item1, e.Item2)].Count > 2)))
            .OrderBy(p => p.Sum(t => m.Normal(m.T[t]).Length))
            .ToList();

        var removed = new bool[m.T.Count];
        var count = 0;
        foreach (var patch in patches)
        {
            var rest = Enumerable.Range(0, m.T.Count).Where(t => !removed[t]).Except(patch).ToList();
            var samples = patch.Take(3).Select(t => (m.P[m.T[t].A] + m.P[m.T[t].B] + m.P[m.T[t].C]) / 3);
            if (rest.Count == 0 || !samples.All(p => Inside(m, rest, p)))
                continue;
            foreach (var t in patch)
                removed[t] = true;
            count++;
        }
        if (count > 0)
            m.Keep(t => !removed[t]);
        return count;
    }

    /// <summary>
    /// Makes neighbouring triangles agree on their winding (each shared edge used once each way); in each piece
    /// the winding most of its triangles had wins. Returns the triangles turned over.
    /// </summary>
    private static int Orient(Mesh m)
    {
        var edges = m.Edges();
        var seen = new bool[m.T.Count];
        var flipped = 0;
        for (var seed = 0; seed < m.T.Count; seed++)
        {
            if (seen[seed])
                continue;
            seen[seed] = true;
            var queue = new Queue<int>([seed]);
            var piece = new List<int>();
            var turned = new List<int>();
            while (queue.Count > 0)
            {
                var t = queue.Dequeue();
                piece.Add(t);
                foreach (var (a, b) in m.T[t].Directed())
                {
                    var users = edges[Mesh.Key(a, b)];
                    if (users.Count != 2)
                        continue;
                    var n = users[0] == t ? users[1] : users[0];
                    if (seen[n])
                        continue;
                    seen[n] = true;
                    if (m.T[n].Directed().Contains((a, b)))
                    {
                        m.T[n] = m.T[n].Flipped;
                        turned.Add(n);
                    }
                    queue.Enqueue(n);
                }
            }
            if (turned.Count * 2 > piece.Count)
            {
                foreach (var t in piece)
                    m.T[t] = m.T[t].Flipped;
                turned = piece.Except(turned).ToList();
            }
            flipped += turned.Count;
        }
        return flipped;
    }

    /// <summary>Caps every open border loop: ear-clipped where it projects flat, else a fan around its centre.</summary>
    private static int FillHoles(Mesh m)
    {
        var edges = m.Edges();
        var outgoing = new Dictionary<int, List<int>>();
        var owner = new Dictionary<(int, int), int>();
        foreach (var tris in edges.Values.Where(l => l.Count == 1))
        {
            var t = tris[0];
            foreach (var (a, b) in m.T[t].Directed())
            {
                if (edges[Mesh.Key(a, b)].Count != 1)
                    continue;
                if (!outgoing.TryGetValue(a, out var list))
                    outgoing[a] = list = [];
                list.Add(b);
                owner[(a, b)] = t;
            }
        }

        var used = new HashSet<(int, int)>();
        var loops = new List<List<int>>();
        foreach (var (start, firsts) in outgoing)
        {
            foreach (var first in firsts)
            {
                if (used.Contains((start, first)))
                    continue;
                // Walk the border; a vertex met twice closes a loop there (pinched borders give several loops).
                var path = new List<int> { start };
                var at = new Dictionary<int, int> { [start] = 0 };
                var (a, b) = (start, first);
                while (used.Add((a, b)))
                {
                    if (at.TryGetValue(b, out var k))
                    {
                        loops.Add(path.GetRange(k, path.Count - k));
                        foreach (var v in path.Skip(k + 1))
                            at.Remove(v);
                        path.RemoveRange(k + 1, path.Count - k - 1);
                    }
                    else
                    {
                        at[b] = path.Count;
                        path.Add(b);
                    }
                    var next = outgoing.GetValueOrDefault(b)?.FirstOrDefault(c => !used.Contains((b, c)), -1) ?? -1;
                    if (next < 0)
                        break;
                    (a, b) = (b, next);
                }
            }
        }

        var filled = 0;
        foreach (var loop in loops.Where(l => l.Count >= 3))
        {
            // The cap runs against the border: the mesh uses a → b, the cap b → a.
            var ring = Enumerable.Reverse(loop).ToList();
            var material = m.M[owner[(loop[0], loop[1])]];
            foreach (var t in Cap(m, ring))
            {
                m.T.Add(t);
                m.M.Add(material);
            }
            filled++;
        }
        return filled;
    }

    private static List<Tri> Cap(Mesh m, List<int> ring)
    {
        if (ring.Count == 3)
            return [new Tri(ring[0], ring[1], ring[2])];
        var points = ring.Select(v => m.P[v]).ToList();
        var normal = Polygon.Normal(points);
        var idx = normal.IsZero(1e-12) ? [] : Polygon.Triangulate(points);
        var tris = new List<Tri>();
        for (var i = 0; i + 2 < idx.Count; i += 3)
            tris.Add(new Tri(ring[idx[i]], ring[idx[i + 1]], ring[idx[i + 2]]));
        if (tris.Count == ring.Count - 2 && tris.All(t => m.Normal(t).Dot(normal) > 0))
            return tris;

        var center = m.P.Count;
        m.P.Add(points.Aggregate(Vec3.Zero, (s, p) => s + p) / points.Count);
        return ring.Select((v, i) => new Tri(v, ring[(i + 1) % ring.Count], center)).ToList();
    }

    /// <summary>
    /// Erases pieces whose bounding box is under <paramref name="fraction"/> of the largest piece's and touches no
    /// other piece's (small details on a part stay).
    /// </summary>
    private static int RemoveLoosePieces(Mesh m, double fraction)
    {
        var pieces = m.Components().Select(tris =>
            (Tris: tris, Bounds: Bounds3.FromPoints(tris.SelectMany(t => new[] { m.P[m.T[t].A], m.P[m.T[t].B], m.P[m.T[t].C] })))).ToList();
        if (pieces.Count < 2)
            return 0;
        var largest = pieces.Max(p => p.Bounds.Diagonal);
        var loose = pieces.Where(p => p.Bounds.Diagonal < fraction * largest
            && !pieces.Any(o => o.Tris != p.Tris && o.Bounds.Diagonal >= fraction * largest && Overlap(o.Bounds, p.Bounds))).ToList();
        var drop = loose.SelectMany(p => p.Tris).ToHashSet();
        m.Keep(t => !drop.Contains(t));
        return loose.Count;
    }

    /// <summary>
    /// Splits the mesh into shells and groups each solid part with its cavities. A closed shell inside another
    /// is a cavity when it faces inwards and a part of its own (an insert, a pin) when it faces outwards; an
    /// inside-out shell inside nothing is turned the right way. Cavities go when asked.
    /// </summary>
    private static List<List<int>> Shells(Mesh m, MeshRepairOptions o, MeshRepairReport report)
    {
        var shells = m.Components().Select(tris => new Shell(tris, m.IsClosed(tris),
            Bounds3.FromPoints(tris.SelectMany(t => new[] { m.P[m.T[t].A], m.P[m.T[t].B], m.P[m.T[t].C] })))).ToList();
        var closedShells = shells.Where(s => s.Closed).ToList();
        foreach (var s in closedShells)
        {
            var samples = s.Tris.Take(3).Select(t => (m.P[m.T[t].A] + m.P[m.T[t].B] + m.P[m.T[t].C]) / 3).ToList();
            s.Containers.AddRange(closedShells.Where(c => c != s && Within(s.Bounds, c.Bounds) && samples.All(p => Inside(m, c.Tris, p))));
            var volume = s.Tris.Sum(t => m.P[m.T[t].A].Dot(m.P[m.T[t].B].Cross(m.P[m.T[t].C])));
            if (volume >= 0)
                continue;
            if (s.Containers.Count > 0)
                s.Cavity = true;
            else
            {
                foreach (var t in s.Tris)
                    m.T[t] = m.T[t].Flipped;
                report.Flipped += s.Tris.Count;
            }
        }
        foreach (var s in closedShells.Where(s => s.Cavity))
            s.Parent = s.Containers.Where(c => !c.Cavity).MinBy(c => c.Bounds.Diagonal);
        if (o.RemoveCavities)
        {
            var cavities = closedShells.Where(s => s.Cavity).ToList();
            report.Cavities = cavities.Count;
            shells = shells.Except(cavities).ToList();
        }

        var bodies = shells.Where(s => !s.Cavity || s.Parent == null)
            .Select(s => s.Tris.Concat(shells.Where(c => c.Cavity && c.Parent == s).SelectMany(c => c.Tris)).ToList())
            .ToList();
        report.OpenShells = shells.Count(s => !s.Closed);
        var keep = new HashSet<int>(shells.SelectMany(s => s.Tris));
        // Bodies refer to the triangle numbers after the erased shells are dropped.
        var renumber = new int[m.T.Count];
        var next = 0;
        for (var i = 0; i < m.T.Count; i++)
            renumber[i] = keep.Contains(i) ? next++ : -1;
        m.Keep(keep.Contains);
        return bodies.Select(b => b.Select(t => renumber[t]).ToList()).ToList();
    }

    private sealed class Shell(List<int> tris, bool closed, Bounds3 bounds)
    {
        public List<int> Tris { get; } = tris;
        public bool Closed { get; } = closed;
        public Bounds3 Bounds { get; } = bounds;
        public List<Shell> Containers { get; } = [];
        public bool Cavity { get; set; }
        public Shell? Parent { get; set; }
    }

    private static bool Overlap(Bounds3 a, Bounds3 b) =>
        a.Min.X <= b.Max.X && b.Min.X <= a.Max.X && a.Min.Y <= b.Max.Y && b.Min.Y <= a.Max.Y && a.Min.Z <= b.Max.Z && b.Min.Z <= a.Max.Z;

    private static bool Within(Bounds3 inner, Bounds3 outer) =>
        inner.Min.X >= outer.Min.X && inner.Min.Y >= outer.Min.Y && inner.Min.Z >= outer.Min.Z
        && inner.Max.X <= outer.Max.X && inner.Max.Y <= outer.Max.Y && inner.Max.Z <= outer.Max.Z
        && inner != outer;

    /// <summary>Point in a closed shell: a ray crossing it an odd number of times.</summary>
    private static bool Inside(Mesh m, List<int> tris, Vec3 p)
    {
        // An odd direction, so the ray does not run along edges of axis-aligned models.
        var d = new Vec3(0.5773, 0.5791, 0.5759).Normalized();
        var hits = 0;
        foreach (var i in tris)
        {
            var t = m.T[i];
            var (a, b, c) = (m.P[t.A], m.P[t.B], m.P[t.C]);
            var e1 = b - a;
            var e2 = c - a;
            var h = d.Cross(e2);
            var det = e1.Dot(h);
            if (Math.Abs(det) < 1e-15)
                continue;
            var s = p - a;
            var u = s.Dot(h) / det;
            if (u < 0 || u > 1)
                continue;
            var q = s.Cross(e1);
            var v = d.Dot(q) / det;
            if (v < 0 || u + v > 1)
                continue;
            if (e2.Dot(q) / det > 1e-9)
                hits++;
        }
        return hits % 2 == 1;
    }

    private static void CountOpenEdges(Mesh m, MeshRepairReport report)
    {
        foreach (var tris in m.Edges().Values)
        {
            if (tris.Count == 1)
                report.BoundaryEdges++;
            else if (tris.Count > 2)
                report.NonManifoldEdges++;
        }
    }

    private static Result Compact(Mesh m, List<List<int>> bodies, MeshRepairReport report)
    {
        var map = new Dictionary<int, int>();
        var positions = new List<Vec3>();
        var tris = new List<int>(m.T.Count * 3);
        foreach (var t in m.T)
        {
            foreach (var v in new[] { t.A, t.B, t.C })
            {
                if (!map.TryGetValue(v, out var id))
                {
                    map[v] = id = positions.Count;
                    positions.Add(m.P[v]);
                }
                tris.Add(id);
            }
        }
        return new Result(positions, tris, m.M, bodies, report);
    }
}
