using Dogeometric.Core.Geometry;
using Dogeometric.Core.Modeling;

namespace Dogeometric.Core.Tests;

public class LockTests
{
    private static (Document Doc, ComponentInstance Group) Locked()
    {
        var doc = new Document(new Model());
        var def = new ComponentDefinition { Name = "G", IsGroup = true };
        TestModels.Box(def.Entities, Vec3.Zero, new Vec3(100, 100, 100));
        doc.Model.Definitions.Add(def);
        var group = doc.Context.Entities.AddInstance(def, Transform.Identity);
        group.Locked = true;
        return (doc, group);
    }

    [Fact]
    public void A_locked_group_is_not_erased_by_Delete()
    {
        var (doc, group) = Locked();
        doc.Selection.Set([group]);
        doc.EraseSelection();
        Assert.Contains(group, doc.Context.Entities.Instances);
    }

    [Fact]
    public void A_locked_group_does_not_open_for_editing()
    {
        var (doc, group) = Locked();
        doc.Edit(group);
        Assert.Empty(doc.Context.Path);
    }
}
