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
        MaterialPropertyBlock mpb;

        GraphicsBuffer instanceBuffer;
        GraphicsBuffer cmdBuffer;
        GraphicsBuffer.IndirectDrawIndexedArgs[] cmdData;

        // TODO: Make all of these directly accessible
        public void UpdateLayout (DDGIManager manager, int3 probeDimensions, float probeDensity, Vector3 boxCenter, Vector3 boxSize, int irrResPad, int irrRes)
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

            var instanceData = new NativeArray<ProbeData>(probeCount, Allocator.Temp);
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

                        instanceData[i++] = new ProbeData()
                        {
                            position = (float3)center,
                            index = new int3(x, y, z),
                        };
                    }
                }
            }

            mpb.Clear();
            instanceBuffer = new GraphicsBuffer(GraphicsBuffer.Target.Structured, probeCount, ProbeData.SIZE);
            instanceBuffer.SetData(instanceData);
            instanceData.Dispose();
            mpb.SetBuffer(ShaderIds._probeData, instanceBuffer);
            mpb.SetTexture(ShaderIds._Irradiance, manager.IrradianceTexture);
            mpb.SetFloat(ShaderIds.irrResPad, irrResPad);
            mpb.SetFloat(ShaderIds.irrRes, irrRes);
        }

        private void OnEnable()
        {

#if UNITY_EDITOR
            SceneView.duringSceneGui -= OnSceneGUI;
            SceneView.duringSceneGui += OnSceneGUI;
#endif
        }

        private void OnDisable()
        {

#if UNITY_EDITOR
            SceneView.duringSceneGui -= OnSceneGUI;
#endif
            instanceBuffer?.Release();
            cmdBuffer?.Release();
            cmdBuffer = null;
        }

#if UNITY_EDITOR
        private void OnSceneGUI(SceneView sceneView)
        {

            Draw(sceneView.camera);
        }
#endif

        private void Draw(Camera camera)
        {
            if (camera == null) return;
            if (debugMaterial == null) return;
            if (cmdBuffer == null) return;

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
    }
}