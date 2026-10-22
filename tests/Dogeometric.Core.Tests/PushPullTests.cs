using Dogeometric.Core.Geometry;
using Dogeometric.Core.Modeling;

namespace Dogeometric.Core.Tests;

public class PushPullTests
{
    [Fact]
    public void Pulling_a_square_makes_a_closed_box()
    {
        var e = new Entities();
        var face = e.AddFace([new Vec3(0, 0, 0), new Vec3(100, 0, 0), new Vec3(100, 100, 0), new Vec3(0, 100, 0)]);
        PushPull.Apply(e, face, face.Normal.Z > 0 ? 50 : -50);
        Assert.Equal(6, e.Faces.Count);
        Assert.Equal(12, e.Edges.Count);
        Assert.Equal(new Vec3(100, 100, 50), e.Bounds().Size);
    }

    [Fact]
    public void An_offset_off_the_normal_moves_the_cap_by_exactly_that_offset_and_keeps_every_wall_flat()
    {
        var e = new Entities();
        var face = e.AddFace([new Vec3(0, 0, 0), new Vec3(100, 0, 0), new Vec3(100, 100, 0), new Vec3(0, 100, 0)]);
        var offset = new Vec3(20, 0, 50);
        var cap = PushPull.Apply(e, face, offset);
        Assert.Equal(6, e.Faces.Count);
        Assert.All(e.Faces, f => Assert.All(f.OuterLoop.Points, p => Assert.Equal(0, f.Normal.Dot(p - f.OuterLoop.Points.First()), 6)));
        Assert.Equal(new Vec3(120, 100, 50), e.Bounds().Size);
        Assert.Equal(50, cap.OuterLoop.Points.First().Z, 6);
    }

    [Fact]
    public void A_pocket_pushed_flush_against_another_shares_the_wall_between_them_away()
    {
        var e = new Entities();
        TestModels.Box(e, Vec3.Zero, new Vec3(100, 100, 100));
        foreach (var (x0, x1) in new[] { (20.0, 50.0), (50.0, 80.0) })
            StickyGeometry.DrawEdges(e, [new Vec3(x0, 30, 100), new Vec3(x1, 30, 100), new Vec3(x1, 70, 100), new Vec3(x0, 70, 100)], closed: true, Vec3.UnitZ);
        foreach (var (x0, x1) in new[] { (20.0, 50.0), (50.0, 80.0) })
        {
            var square = e.Faces.Single(f => f.Normal.Dot(Vec3.UnitZ) > 0.99 && f.Area < 2000 && f.OuterLoop.Points.All(p => p.X >= x0 - 1e-6 && p.X <= x1 + 1e-6));
            PushPull.Apply(e, square, -30);
        }
        var bad = string.Join("\n", e.Edges.Where(x => Topology.FacesOf(e, x).Count() != 2).Select(x => $"DBG edge {x.Start.Position}->{x.End.Position}: " + string.Join(" | ", Topology.FacesOf(e, x).Select(f => $"n={f.Normal} area={f.Area:0.#}"))));
        Assert.True(bad.Length == 0, bad);
        Assert.DoesNotContain(e.Faces, f => Math.Abs(f.Normal.X) > 0.99 && f.OuterLoop.Points.All(p => Math.Abs(p.X - 50) < 1e-6));
        Assert.Equal(2400, e.Faces.Where(f => f.Normal.Z > 0.99 && Math.Abs(f.OuterLoop.Points.First().Z - 70) < 1e-6).Sum(f => f.Area), 3);
    }

    [Fact]
    public void Pushing_an_inner_square_to_the_far_face_cuts_a_hole_through()
    {
        // A 100 box, then a 40 square drawn on its top, pushed down the whole height.
        var e = new Entities();
        TestModels.Box(e, Vec3.Zero, new Vec3(100, 100, 100));
        var top = e.Faces.Single(f => f.Normal.Dot(Vec3.UnitZ) > 0.99);
        StickyGeometry.DrawEdges(e, [new Vec3(30, 30, 100), new Vec3(70, 30, 100), new Vec3(70, 70, 100), new Vec3(30, 70, 100)], closed: true, Vec3.UnitZ);
        var inner = e.Faces.Single(f => f.Normal.Dot(Vec3.UnitZ) > 0.99 && f.Area < 2000);
        PushPull.Apply(e, inner, -100);

        // Neither a cap at the top nor a face across the hole at the bottom remains.
        Assert.DoesNotContain(e.Faces, f => Math.Abs(f.Normal.Z) > 0.99 && f.Area < 2000);
        var bottom = e.Faces.Single(f => f.Normal.Dot(-Vec3.UnitZ) > 0.99);
        Assert.Equal(100 * 100 - 40 * 40, bottom.Area, 6);
        Assert.Equal(2, bottom.Loops.Count);
        // Four walls line the hole.
        Assert.Equal(4, e.Faces.Count(f => Math.Abs(f.Normal.Z) < 1e-9 && f.Area == 40 * 100));
    }

    [Fact]
    public void Pushing_a_box_side_just_moves_it()
    {
        var e = new Entities();
        TestModels.Box(e, Vec3.Zero, new Vec3(100, 100, 100));
        var side = e.Faces.Single(f => f.Normal.Dot(Vec3.UnitX) > 0.99);
        PushPull.Apply(e, side, 50);
        Assert.Equal(6, e.Faces.Count);
        Assert.Equal(new Vec3(150, 100, 100), e.Bounds().Size);
    }

    [Fact]
    public void Pulling_the_end_of_a_bar_past_the_top_it_stands_on_keeps_a_solid()
    {
        // A 100×50×100 box with a 60×30 bar pushed up 30 from a corner of its top, then the bar's end pulled
        // 40 out past the back: the underside of the overhang must not fold the box's top over itself.
        var e = new Entities();
        TestModels.Box(e, Vec3.Zero, new Vec3(100, 50, 100));
        StickyGeometry.DrawEdges(e, [new Vec3(40, 0, 100), new Vec3(40, 30, 100), new Vec3(100, 30, 100)], closed: false, Vec3.UnitZ);
        var strip = e.Faces.Single(f => f.Normal.Dot(Vec3.UnitZ) > 0.99 && f.Area == 60 * 30);
        PushPull.Apply(e, strip, 30);
        var end = e.Faces.Single(f => f.Normal.Dot(Vec3.UnitY) > 0.99 && f.Area == 60 * 30);
        PushPull.Apply(e, end, 40);

        Assert.Empty(SolidInspector.Find(e));
        Assert.Equal(40 * 50, e.Faces.Single(f => f.Normal.Dot(Vec3.UnitZ) > 0.99 && f.OuterLoop.Points.All(p => p.Z == 100)).Area, 6);
        Assert.Equal(60 * 20, e.Faces.Single(f => f.Normal.Dot(-Vec3.UnitZ) > 0.99 && f.OuterLoop.Points.All(p => p.Z == 100)).Area, 6);
        Assert.Equal(new Vec3(100, 70, 130), e.Bounds().Size);
    }

    [Fact]
    public void Pulling_a_pocket_floor_up_shrinks_its_walls()
    {
        var e = new Entities();
        TestModels.Box(e, Vec3.Zero, new Vec3(100, 100, 100));
        StickyGeometry.DrawEdges(e, [new Vec3(30, 30, 100), new Vec3(70, 30, 100), new Vec3(70, 70, 100), new Vec3(30, 70, 100)], closed: true, Vec3.UnitZ);
        var inner = e.Faces.Single(f => f.Normal.Dot(Vec3.UnitZ) > 0.99 && f.Area < 2000);
        var floor = PushPull.Apply(e, inner, -50);
        PushPull.Apply(e, floor, 20);

        Assert.Empty(SolidInspector.Find(e));
        Assert.Equal(4, e.Faces.Count(f => Math.Abs(f.Normal.Z) < 1e-9 && Math.Abs(f.Area - 40 * 30) < 1e-6));
    }
}
