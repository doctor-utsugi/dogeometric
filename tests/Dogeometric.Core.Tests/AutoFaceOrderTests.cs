using Dogeometric.Core.Geometry;
using Dogeometric.Core.Modeling;

namespace Dogeometric.Core.Tests;

public class AutoFaceOrderTests
{
    private static void Chain(Entities e, params Vec3[] points)
    {
        for (var i = 0; i < points.Length - 1; i++)
            StickyGeometry.DrawEdges(e, [points[i], points[i + 1]]);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void A_square_around_another_has_the_inner_one_as_a_hole_whichever_is_drawn_first(bool innerFirst)
    {
        var e = new Entities();
        Vec3[] Square(double lo, double hi) => [new(lo, lo, 0), new(hi, lo, 0), new(hi, hi, 0), new(lo, hi, 0), new(lo, lo, 0)];
        if (innerFirst)
        {
            Chain(e, Square(35, 65));
            Chain(e, Square(0, 100));
        }
        else
        {
            Chain(e, Square(0, 100));
            Chain(e, Square(35, 65));
        }
        Assert.Equal(2, e.Faces.Count);
        var outer = e.Faces.Single(f => f.Area > 5000 || f.InnerLoops.Any());
        Assert.Single(outer.InnerLoops);
        Assert.Equal(100 * 100 - 30 * 30, outer.Area, 3);
    }
}
