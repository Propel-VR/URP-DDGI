using Unity.Collections;
using Unity.Mathematics;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;

namespace DDGIURP
{
    [ExecuteInEditMode]
    public class DDGIDebugProbeRenderer : MonoBehaviour
    {
        [SerializeField] Material debugMaterial;
        [SerializeField] Mesh debugSphereMesh;

        Vector3 boxCenter;
        Vector3 boxSize;
        int probeCount;
        NativeArray<float3> instanceData;
        MaterialPropertyBlock mpb;

        GraphicsBuffer instanceBuffer;
        GraphicsBuffer cmdBuffer;
        GraphicsBuffer.IndirectDrawIndexedArgs[] cmdData;

        void OnDestroy ()
        {
            instanceBuffer?.Release();
            cmdBuffer?.Release();
            cmdBuffer = null;
        }

        public void UpdateLayout (int3 probeDimensions, float probeDensity, Vector3 boxCenter, Vector3 boxSize)
        {
            this.boxCenter = boxCenter;
            this.boxSize = boxSize;

            instanceBuffer?.Release();


            if (cmdBuffer == null || mpb == null || cmdData == null)
            {
                mpb = new MaterialPropertyBlock();
                cmdBuffer = new GraphicsBuffer(GraphicsBuffer.Target.IndirectArguments, 1, GraphicsBuffer.IndirectDrawIndexedArgs.size);
                cmdData = new GraphicsBuffer.IndirectDrawIndexedArgs[1];
            }

            probeCount = probeDimensions.x * probeDimensions.y * probeDimensions.z;
            cmdData[0].instanceCount = (uint)probeCount;
            cmdData[0].indexCountPerInstance = debugSphereMesh.GetIndexCount(0);
            cmdBuffer.SetData(cmdData);
            int i = 0;

            instanceData = new NativeArray<float3>(probeCount, Allocator.Temp);
            for (int x = 0; x < probeDimensions.x; x++)
            {
                for (int y = 0; y < probeDimensions.y; y++)
                {
                    for (int z = 0; z < probeDimensions.z; z++)
                    {
                        var center = new Vector3(x, y, z) * probeDensity +
                            boxCenter - boxSize * 0.5f +
                            Vector3.one * (probeDensity * 0.5f) +
                            transform.position;

                        instanceData[i++] = (float3)center;
                    }
                }
            }

            mpb.Clear();
            instanceBuffer = new GraphicsBuffer(GraphicsBuffer.Target.Structured, probeCount, sizeof(float) * 3 * 4);
            instanceBuffer.SetData(instanceData);
            instanceData.Dispose();
            mpb.SetBuffer("_positionData", instanceBuffer);
        }

#if UNITY_EDITOR
        private void OnEnable()
        {

            SceneView.duringSceneGui -= OnSceneGUI;
            SceneView.duringSceneGui += OnSceneGUI;
        }

        private void OnDisable()
        {

            SceneView.duringSceneGui -= OnSceneGUI;
        }

        private void OnSceneGUI(SceneView sceneView)
        {

            Draw(sceneView.camera);
        }

        private void Draw(Camera camera)
        {
            if (camera == null) return;
            if (debugMaterial == null) return;

            var renderParams = new RenderParams(debugMaterial)
            {
                camera = camera,
                receiveShadows = false,
                shadowCastingMode = ShadowCastingMode.Off,
                worldBounds = new Bounds(boxCenter + transform.position, boxSize),
                matProps = mpb,
                renderingLayerMask = uint.MaxValue,
            };
            Graphics.RenderMeshIndirect(
                renderParams,
                mesh: debugSphereMesh,
                cmdBuffer
            );
        }
#endif
    }
}