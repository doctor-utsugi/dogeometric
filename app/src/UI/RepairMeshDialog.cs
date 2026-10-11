using Dogeometric.Core.IO;
using Godot;

namespace Dogeometric.App.UI;

/// <summary>Tools › Solid Tools › Repair Mesh: the options, then a summary of what was fixed and what is still open.</summary>
public static class RepairMeshDialog
{
    /// <summary>The options of the last repair, offered again next time.</summary>
    public static MeshRepairOptions Last { get; set; } = new();

    public static void Show(Node parent, Action<MeshRepairOptions> repair)
    {
        var o = Last;
        var d = new ConfirmationDialog { Title = "Repair Mesh", OkButtonText = "Repair", CancelButtonText = "Cancel" };
        var box = new VBoxContainer { CustomMinimumSize = new Vector2(380, 0) };
        box.AddThemeConstantOverride("separation", 4);

        SpinBox Number(string text, double value, double max, double step, string suffix, string tip)
        {
            var row = new HBoxContainer { TooltipText = tip };
            row.AddChild(new Label { Text = text, SizeFlagsHorizontal = Control.SizeFlags.ExpandFill });
            var spin = new SpinBox { MinValue = 0, MaxValue = max, Step = step, Value = value, Suffix = suffix, CustomMinimumSize = new Vector2(120, 0) };
            d.RegisterTextEnter(spin.GetLineEdit());
            row.AddChild(spin);
            box.AddChild(row);
            return spin;
        }
        CheckBox Check(string text, bool value, string tip)
        {
            var c = new CheckBox { Text = text, ButtonPressed = value, TooltipText = tip };
            box.AddChild(c);
            return c;
        }

        var gap = Number("Stitch gaps up to", o.GapTolerance, 5, 0.001, "mm", "Open borders closer than this are sewn together.");
        var holes = Check("Fill holes", o.FillHoles, "Caps every open border left after stitching.");
        var loose = Number("Erase loose pieces under", o.LoosePieceFraction * 100, 50, 0.1, "%",
            "Pieces smaller than this share of the largest piece's size are erased (0 keeps all).");
        var cavities = Check("Erase cavities", o.RemoveCavities, "Shells inside other shells go, so hollow parts become filled.");
        var merge = Check("Merge overlapping shells", o.MergeShells, "Shells that cross each other become one solid.");

        d.AddChild(box);
        d.Confirmed += () =>
        {
            Last = new MeshRepairOptions
            {
                GapTolerance = gap.Value,
                FillHoles = holes.ButtonPressed,
                LoosePieceFraction = loose.Value / 100,
                RemoveCavities = cavities.ButtonPressed,
                MergeShells = merge.ButtonPressed,
            };
            d.QueueFree();
            repair(Last);
        };
        d.Canceled += d.QueueFree;
        parent.AddChild(d);
        d.PopupCentered();
    }

    public static void Summary(Node parent, IReadOnlyList<MeshRepairReport> reports, TimeSpan elapsed)
    {
        var lines = new List<string>();
        foreach (var (label, count) in reports.SelectMany(r => r.Lines()).GroupBy(l => l.Label, l => l.Count)
                     .Select(g => (g.Key, g.Sum())).Where(l => l.Item2 > 0))
            lines.Add($"> {label}: {count}");
        if (lines.Count == 0)
            lines.Add("> Nothing to repair.");
        foreach (var error in reports.Select(r => r.MergeError).OfType<string>().Distinct())
            lines.Add($"> Shells not merged: {error}");
        var open = reports.Where(r => !r.IsClosed).ToList();
        lines.Add("");
        lines.Add(open.Count == 0
            ? "The result is a closed solid."
            : $"Still not a solid: {open.Sum(r => r.BoundaryEdges)} open edges, {open.Sum(r => r.NonManifoldEdges)} edges shared by more than two faces.");
        lines.Add($"> Total Elapsed Time: {elapsed.TotalSeconds:0.000}s");
        MessageDialog.Show(parent, "Repair Mesh", string.Join("\n", lines));
    }
}
