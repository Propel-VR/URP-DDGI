using System.Linq;
using UnityEditor;
using UnityEditor.EditorTools;
using UnityEditor.Overlays;
using UnityEngine;
using UnityEngine.UIElements;

namespace DDGIURP
{
    [EditorTool("DDGI Tool", typeof(DDGIManager)), Icon("d_debug")]
    public class DDGITool : EditorTool
    {
        [Overlay(displayName = "DDGI Debug Overlay", defaultDisplay = true), Icon("LightProbeProxyVolume Gizmo")]
        class DDGIOverlay : Overlay
        {
            internal DDGIManager _manager;

            public override VisualElement CreatePanelContent()
            {
                var root = new VisualElement();

                var tabView = new TabView();
                var tab1 = new Tab("Radiance");
                var tab2 = new Tab("Visibility");
                
                if(_manager != null)
                {
                    tab1.Add(new UnityEngine.UIElements.Image() 
                    {
                        image = _manager.IrradianceTexture,
                    });
                    tab2.Add(new UnityEngine.UIElements.Image()
                    {
                        image = _manager.VisibilityTexture,
                    });
                }
                else
                {
                    root.Add(new Label("No DDGI Manager Selected"));
                }
                
                tabView.Add(tab1);
                tabView.Add(tab2);
                root.Add(tabView);
                return root;

            }
        }

        DDGIOverlay _overlay;

        public override void OnActivated()
        {
            SceneView.AddOverlayToActiveView(_overlay = new DDGIOverlay() { _manager = target as DDGIManager });
        }

        public override void OnWillBeDeactivated()
        {
            SceneView.RemoveOverlayFromActiveView(_overlay);
        }
    }
}
