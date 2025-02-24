using UnityEngine;
using UnityEngine.Rendering;
using System.Collections.Generic;
using System.Linq;

namespace DDGIURP { 
    public static class DDGIDebugPanel
    {
        public class DebugPanelSettings
        {
            public bool voxelEnabled = false;

        }
        static DebugPanelSettings settings = new();

        public static bool VoxelEnabled => settings.voxelEnabled;

        public static void CreatePanel ()
        {
            var widgetList = new List<DebugUI.Widget>();

            widgetList.AddRange(new DebugUI.Widget[]
            {
                new DebugUI.Foldout("Voxels", new ObservableList<DebugUI.Widget>()
                {
                    new DebugUI.BoolField
                    {
                        displayName = "Toggle Voxel View",
                        tooltip = "Enable or disable the main directional light",
                        getter = () => settings.voxelEnabled,
                        setter = value => settings.voxelEnabled = value,
                        onValueChanged = ToggleVoxelViewCallback
                    }
                })
            });

            var panel = DebugManager.instance.GetPanel("DDGI", createIfNull: true);
            panel.children.Add(widgetList.ToArray());
        }

        public static void RemovePanel ()
        {
            DebugManager.instance.RemovePanel("DDGI");
        }

        static void ToggleVoxelViewCallback (DebugUI.Field<bool> field, bool value)
        {
            // Do something!
        }
    }
}