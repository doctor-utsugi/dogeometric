using Dogeometric.Core.Geometry;
using Dogeometric.Core.IO;
using Dogeometric.Core.Modeling;

namespace Dogeometric.Solids.Tests;

public class MeshRepairTests
{
    static MeshRepairTests() => SolidsNative.AddSearchDirectory(Path.Combine(AppContext.BaseDirectory, "native"));

    /// <summary>The six sides of a box as STL triangles, wound outwards.</summary>
    private static List<Triangle> Box(Vec3 min, Vec3 size)
    {
        Vec3 P(double x, double y, double z) => min + new Vec3(x * size.X, y * size.Y, z * size.Z);
        List<Vec3[]> quads =
        [
            [P(0, 0, 0), P(0, 1, 0), P(1, 1, 0), P(1, 0, 0)],
            [P(0, 0, 1), P(1, 0, 1), P(1, 1, 1), P(0, 1, 1)],
            [P(0, 0, 0), P(1, 0, 0), P(1, 0, 1), P(0, 0, 1)],
            [P(0, 1, 0), P(0, 1, 1), P(1, 1, 1), P(1, 1, 0)],
            [P(0, 0, 0), P(0, 0, 1), P(0, 1, 1), P(0, 1, 0)],
            [P(1, 0, 0), P(1, 1, 0), P(1, 1, 1), P(1, 0, 1)],
        ];
        return quads.SelectMany(q => new[] { new Triangle(q[0], q[1], q[2], null), new Triangle(q[0], q[2], q[3], null) }).ToList();
    }

    private static readonly Vec3 Cube = new(100, 100, 100);

    private static (ComponentInstance? Result, MeshRepairReport Report) Repair(List<Triangle> triangles, MeshRepairOptions? options = null)
    {
        var m = new Model();
        var def = new ComponentDefinition { Name = "mesh", IsGroup = true };
        MeshImport.AddMerged(def.Entities, triangles.Select(t => (new List<Vec3> { t.A, t.B, t.C }, (object?)null)).ToList());
        m.Definitions.Add(def);
        var input = m.Entities.AddInstance(def, Transform.Identity);
        return SolidTools.RepairMesh(m, m.Entities, input, options ?? new MeshRepairOptions());
    }

    private static MeshCheck.Report Check(ComponentInstance i) => MeshCheck.Analyze(MeshExtractor.ExtractInstance(i));

    [Fact]
    public void Fills_a_missing_side()
    {
        var tris = Box(Vec3.Zero, Cube);
        tris.RemoveRange(2, 2);
        var (result, report) = Repair(tris);
        Assert.Equal(1, report.HolesFilled);
        Assert.True(report.IsClosed);
        var check = Check(result!);
        Assert.True(check.IsWatertight);
        Assert.Equal(1_000_000, check.Volume, 0);
    }

    [Fact]
    public void A_loose_open_sliver_goes_without_being_capped()
    {
        var tris = Box(Vec3.Zero, Cube);
        tris.RemoveAt(4);
        tris.Add(new Triangle(new Vec3(500, 0, 0), new Vec3(500.1, 0, 0), new Vec3(500, 0.1, 0), null));
        var (result, report) = Repair(tris);
        Assert.Equal(1, report.HolesFilled);
        Assert.Equal(1, report.LoosePieces);
        Assert.Equal(1_000_000, Check(result!).Volume, 0);
    }

    [Fact]
    public void Stitches_a_gap_smaller_than_the_tolerance()
    {
        var tris = Box(Vec3.Zero, Cube);
        // The top side floats 0.02 mm above the walls.
        var lift = new Vec3(0, 0, 0.02);
        tris[2] = tris[2] with { A = tris[2].A + lift, B = tris[2].B + lift, C = tris[2].C + lift };
        tris[3] = tris[3] with { A = tris[3].A + lift, B = tris[3].B + lift, C = tris[3].C + lift };
        Assert.False(MeshCheck.Analyze(tris).IsWatertight);

        var (result, report) = Repair(tris);
        Assert.True(report.Stitched > 0);
        Assert.Equal(0, report.HolesFilled);
        Assert.True(Check(result!).IsWatertight);
    }

    [Fact]
    public void Closes_t_junctions_where_one_side_has_an_extra_vertex()
    {
        var tris = Box(Vec3.Zero, Cube);
        // The top side as four triangles around a vertex in the middle of its front edge, which the front wall lacks.
        var (a, b, c, d, mid) = (new Vec3(0, 0, 100), new Vec3(100, 0, 100), new Vec3(100, 100, 100), new Vec3(0, 100, 100), new Vec3(50, 0, 100));
        tris.RemoveRange(2, 2);
        tris.AddRange([new(a, mid, d, null), new(mid, b, c, null), new(mid, c, d, null)]);
        Assert.False(MeshCheck.Analyze(tris).IsWatertight);

        var (result, report) = Repair(tris, new MeshRepairOptions { FillHoles = false });
        Assert.Equal(1, report.TJunctions);
        Assert.True(Check(result!).IsWatertight);
    }

    [Fact]
    public void Turns_reversed_triangles_and_inside_out_shells()
    {
        var tris = Box(Vec3.Zero, Cube).Select(t => new Triangle(t.A, t.C, t.B, null)).ToList();
        tris[5] = new Triangle(tris[5].A, tris[5].C, tris[5].B, null);
        var (result, report) = Repair(tris);
        // One against its neighbours, then the whole shell.
        Assert.Equal(13, report.Flipped);
        var check = Check(result!);
        Assert.True(check.IsWatertight);
        Assert.Equal(1_000_000, check.Volume, 0);
    }

    [Fact]
    public void Erases_duplicates_degenerates_and_loose_bits()
    {
        var tris = Box(Vec3.Zero, Cube);
        tris.Add(tris[0]);
        tris.Add(new Triangle(new Vec3(0, 0, 0), new Vec3(50, 0, 0), new Vec3(100, 0, 0), null));
        tris.AddRange(Box(new Vec3(500, 0, 0), new Vec3(0.5, 0.5, 0.5)));
        var raw = MeshRepair.Run(tris);
        Assert.Equal(1, raw.Report.Duplicates);
        Assert.Equal(1, raw.Report.Degenerate);
        Assert.Equal(1, raw.Report.LoosePieces);
        Assert.Equal(12, raw.Triangles.Count / 3);

        var (result, _) = Repair(tris);
        var check = Check(result!);
        Assert.True(check.IsWatertight);
        Assert.Equal(1_000_000, check.Volume, 0);
    }

    [Fact]
    public void An_inward_shell_inside_another_is_its_cavity_or_goes()
    {
        var inner = Box(new Vec3(25, 25, 25), new Vec3(50, 50, 50)).Select(t => new Triangle(t.A, t.C, t.B, null));
        var tris = Box(Vec3.Zero, Cube).Concat(inner).ToList();
        var (hollow, _) = Repair(tris);
        var check = Check(hollow!);
        Assert.True(check.IsWatertight);
        Assert.Equal(1_000_000 - 125_000, check.Volume, 0);

        var (filled, report) = Repair(tris, new MeshRepairOptions { RemoveCavities = true });
        Assert.Equal(1, report.Cavities);
        Assert.Equal(1_000_000, Check(filled!).Volume, 0);
    }

    [Fact]
    public void An_outward_shell_inside_another_is_a_part_and_merges()
    {
        // An insert modelled inside its housing, as assemblies export.
        var tris = Box(Vec3.Zero, Cube).Concat(Box(new Vec3(25, 25, 25), new Vec3(50, 50, 50))).ToList();
        var (result, report) = Repair(tris);
        Assert.Equal(0, report.Flipped);
        Assert.Equal(2, report.ShellsMerged);
        var check = Check(result!);
        Assert.True(check.IsWatertight);
        Assert.InRange(check.Volume, 1_000_000, 1_001_000);
    }

    [Fact]
    public void Parts_touching_along_an_edge_stay_apart_until_merged()
    {
        var tris = Box(Vec3.Zero, Cube).Concat(Box(new Vec3(100, 100, 0), Cube)).ToList();
        var raw = MeshRepair.Run(tris);
        Assert.Equal(2, raw.Bodies.Count);
        Assert.Equal(0, raw.Report.OpenShells);
        Assert.Equal(0, raw.Report.Flipped);
        Assert.Equal(0, raw.Report.InternalWalls);
    }

    [Fact]
    public void Parts_touching_along_an_edge_fuse_when_merged()
    {
        var tris = Box(Vec3.Zero, Cube).Concat(Box(new Vec3(100, 100, 0), Cube)).ToList();
        var (result, report) = Repair(tris);
        Assert.Equal(2, report.ShellsMerged);
        Assert.True(report.IsClosed);
        var check = Check(result!);
        Assert.True(check.IsWatertight);
        Assert.InRange(check.Volume, 2_000_000, 2_002_000);
    }

    [Fact]
    public void Slivers_collapse_without_changing_the_volume()
    {
        var tris = Box(Vec3.Zero, Cube);
        // The top side as a fan around a point 0.0004 mm from a corner: two needle triangles there.
        var (a, b, c, d) = (new Vec3(0, 0, 100), new Vec3(100, 0, 100), new Vec3(100, 100, 100), new Vec3(0, 100, 100));
        var near = new Vec3(0.0004, 0.0003, 100);
        tris.RemoveRange(2, 2);
        tris.AddRange([new(a, b, near, null), new(near, b, c, null), new(near, c, d, null), new(near, d, a, null)]);
        // Indexed without welding, as the boolean engine hands meshes back.
        var positions = new List<Vec3>();
        var triangles = new List<int>();
        foreach (var t in tris)
            foreach (var p in new[] { t.A, t.B, t.C })
            {
                var i = positions.IndexOf(p);
                triangles.Add(i >= 0 ? i : positions.Count);
                if (i < 0)
                    positions.Add(p);
            }
        Assert.True(MeshRepair.RemoveSlivers(positions, triangles) > 0);
        var check = MeshCheck.Analyze(Enumerable.Range(0, triangles.Count / 3)
            .Select(t => new Triangle(positions[triangles[t * 3]], positions[triangles[t * 3 + 1]], positions[triangles[t * 3 + 2]], null)).ToList(), 1e-9);
        Assert.True(check.IsWatertight);
        Assert.InRange(check.Volume, 999_998, 1_000_002);
        for (var t = 0; t < triangles.Count; t += 3)
            Assert.True(positions[triangles[t]].DistanceTo(positions[triangles[t + 1]]) > Tolerance.Length);
    }

    [Fact]
    public void A_small_part_on_a_big_one_is_not_a_loose_piece()
    {
        var tris = Box(Vec3.Zero, Cube).Concat(Box(new Vec3(50, 50, 99.8), new Vec3(0.5, 0.5, 0.5))).ToList();
        Assert.Equal(0, MeshRepair.Run(tris).Report.LoosePieces);
    }

    [Fact]
    public void Overlapping_shells_merge_into_one_solid()
    {
        var tris = Box(Vec3.Zero, Cube).Concat(Box(new Vec3(50, 0, 0), Cube)).ToList();
        var (result, report) = Repair(tris);
        Assert.Null(report.MergeError);
        Assert.Equal(2, report.ShellsMerged);
        var check = Check(result!);
        Assert.True(check.IsWatertight);
        // The parts grow a few microns before the union, so touching ones fuse.
        Assert.InRange(check.Volume, 1_500_000, 1_501_000);
    }

    [Fact]
    public void Boxes_touching_face_to_face_lose_the_wall_between_them()
    {
        var tris = Box(Vec3.Zero, Cube).Concat(Box(new Vec3(100, 0, 0), Cube)).ToList();
        var raw = MeshRepair.Run(tris, new MeshRepairOptions { MergeShells = false });
        Assert.Equal(2, raw.Report.InternalWalls);
        Assert.True(raw.Report.IsClosed);

        // Imported, the two walls are already one face shared by both boxes.
        var (result, report) = Repair(tris, new MeshRepairOptions { MergeShells = false });
        Assert.Equal(1, report.InternalWalls);
        var check = Check(result!);
        Assert.True(check.IsWatertight);
        Assert.Equal(2_000_000, check.Volume, 0);
    }
}
