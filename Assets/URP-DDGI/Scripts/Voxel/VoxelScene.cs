using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Experimental.Rendering;
using EasyButtons;
using System.Collections.Generic;
using System.Linq;
using UnityEngine.Rendering.Universal;
using Unity.Rendering;

namespace DDGIURP
{
    [ExecuteAlways]
    public class VoxelScene : MonoBehaviour
    {
        [Header("Parameters")]
        [SerializeField] int resolution = 256;
        [SerializeField] float scale = 1f;
        [SerializeField] Mesh triangleMesh;
        
        [Header("Debug")]
        [SerializeField] bool debugDrawEveryFrame = false;
        [SerializeField] bool debugDrawVoxel = false;
        [ReadOnly, SerializeField] RenderTexture volume;
        [ReadOnly, SerializeField] RenderTexture target;
        [ReadOnly, SerializeField] List<Renderer> filteredRenderers = new();
        [ReadOnly, SerializeField] List<MeshPair> meshRenderers = new();

        VoxelDebugPass debugPass;
        Shader voxelShader;
        ComputeShader voxelCompute;
        Material voxelMaterial;
        LocalKeyword has32BitIndicesKeyword;
        LocalKeyword hasNormalsKeyword;
        LocalKeyword hasUVsKeyword;
        LocalKeyword instancingKeyword;
        LocalKeyword proceduralKeyword;
        GlobalKeyword instancing2Keyword;
        int clearKernel;
        Matrix4x4[] viewProjectMats = new Matrix4x4[3];
        List<GraphicsBuffer> tempBufferCache = new();
        List<MaterialPropertyBlock> mpbs = new();

        // TODO: How to automatically detect rendereres without requiring a script on them, or a references to them?
        // We want to ensure raytracing is enabled for them. Maybe filter with RenderLayers?
        [Button]
        public void GetAllRenderers ()
        {
            filteredRenderers = FindObjectsByType<Renderer>(FindObjectsSortMode.None)
                .Where((r) => r.rayTracingMode != RayTracingMode.Off).ToList();

            meshRenderers.Clear();
            foreach (var renderer in filteredRenderers)
            {
                if (renderer is MeshRenderer)
                {
                    meshRenderers.Add(
                        new MeshPair(
                            renderer.GetComponent<MeshFilter>().sharedMesh,
                            renderer.GetComponent<MeshRenderer>()));
                }
            }
        }

        private void OnEnable () {
            //RenderPipelineManager.beginCameraRendering += OnBeginCamera;
            UpdateAssets();
            Allocate();
        }

        private void OnValidate () => UpdateAssets();

        private void UpdateAssets ()
        {
            voxelShader = Resources.Load<Shader>("Shaders/VoxelShader");
            voxelCompute = Resources.Load<ComputeShader>("Shaders/Voxel");
            has32BitIndicesKeyword = voxelShader.keywordSpace.FindKeyword("HAS_32_BIT_INDICES");
            hasNormalsKeyword = voxelShader.keywordSpace.FindKeyword("HAS_NORMALS");
            hasUVsKeyword = voxelShader.keywordSpace.FindKeyword("HAS_UVS");
            hasUVsKeyword = voxelShader.keywordSpace.FindKeyword("HAS_UVS");
            hasUVsKeyword = voxelShader.keywordSpace.FindKeyword("HAS_UVS");
            instancingKeyword = voxelShader.keywordSpace.FindKeyword("INSTANCING_ON");
            proceduralKeyword = voxelShader.keywordSpace.FindKeyword("PROCEDURAL_INSTANCING_ON");
            clearKernel = voxelCompute.FindKernel("Clear");
            instancing2Keyword = GlobalKeyword.Create("UNITY_SUPPORT_INSTANCING");
        }

        private void OnDisable () {
            //RenderPipelineManager.beginCameraRendering -= OnBeginCamera;
            Free(); 
        }

        private void OnBeginCamera (ScriptableRenderContext context, Camera cam)
        {
            if(debugDrawVoxel)
            {
                cam.GetUniversalAdditionalCameraData().scriptableRenderer.EnqueuePass(debugPass);
            }
        }

        private void Allocate ()
        {
            volume = new RenderTexture(resolution, resolution, 0, GraphicsFormat.R8G8B8A8_UNorm) // R32_UInt
            {
                dimension = TextureDimension.Tex3D,
                volumeDepth = resolution,
                enableRandomWrite = true
            };
            volume.Create();

            target = new RenderTexture(resolution, resolution, 0, GraphicsFormat.R8_UNorm)
            {
                dimension = TextureDimension.Tex2D,
                enableRandomWrite = true
            };
            target.Create();

            debugPass = new VoxelDebugPass();
            debugPass.SetVoxelTarget(volume, resolution);
            voxelMaterial = new Material(voxelShader);
        }

        private void Free ()
        {
            if (volume != null) DestroyImmediate(volume);
            if (target != null) DestroyImmediate(target);
            if (voxelMaterial != null) DestroyImmediate(voxelMaterial);
        }
         
        [Button]
        public void BakeScene ()
        {
            Free();
            Allocate();

            RenderAll();
        }

        void RenderAll ()
        {
            // Fix issue where shader keyword get broken on recompile
#if UNITY_EDITOR
            UpdateAssets();
#endif

            var cmd = CommandBufferPool.Get("Voxelize Scene");

            // Clear textures
            int threadClear = Mathf.CeilToInt(resolution / 4f);
            cmd.SetComputeTextureParam(voxelCompute, clearKernel, ShaderIds._VoxelTarget, volume);
            cmd.SetComputeIntParam(voxelCompute, ShaderIds.resolution, resolution);
            cmd.DispatchCompute(voxelCompute, clearKernel, threadClear, threadClear, threadClear);

            // Prepare fake cameras
            float extent = resolution*scale/2f;
            Vector3 center = transform.position;
            Vector3 volumeCenter = center - new Vector3(extent, extent, extent);
            cmd.SetRenderTarget(target);
            cmd.ClearRenderTarget(true, true, Color.black);

            // Orthographic projection matrix
            var camOrtho = GL.GetGPUProjectionMatrix(Matrix4x4.Ortho(-extent, extent, -extent, extent, 0, extent * 2), renderIntoTexture: true);
            var camTRS0 = Matrix4x4.TRS(new Vector3(center.x + extent, center.y, center.z), Quaternion.Euler(0, -90, 0), new Vector3(1, 1, -1));
            var camTRS1 = Matrix4x4.TRS(new Vector3(center.x, center.y + extent, center.z), Quaternion.Euler(90, 0, 0), new Vector3(1, 1, -1));
            var camTRS2 = Matrix4x4.TRS(new Vector3(center.x, center.y, center.z + extent), Quaternion.Euler(0, 180, 0), new Vector3(1, 1, -1));
            viewProjectMats[0] = camOrtho * camTRS0.inverse;
            viewProjectMats[1] = camOrtho * camTRS1.inverse;
            viewProjectMats[2] = camOrtho * camTRS2.inverse;

            // Setup shared values
            voxelMaterial.SetMatrixArray(ShaderIds.ViewProjectionMatrix, viewProjectMats);
            voxelMaterial.SetTexture(ShaderIds._VoxelTarget, volume);
            voxelMaterial.SetInt(ShaderIds.resolution, resolution);
            voxelMaterial.SetFloat(ShaderIds.invScale, 1f / scale);
            voxelMaterial.SetVector(ShaderIds.VolumeCenter, new Vector4(volumeCenter.x, volumeCenter.y, volumeCenter.z, 0));

            // Draw all renderers
            tempBufferCache.Clear();
            mpbs.Clear();
            int l = 0;
            foreach (var pair in meshRenderers)
            {
                var mesh = pair.mesh;
                var renderer = pair.meshRenderer;
                var tr = renderer.transform;
                int indexCount = 0;
                mesh.vertexBufferTarget |= GraphicsBuffer.Target.Raw;
                mesh.indexBufferTarget |= GraphicsBuffer.Target.Raw;
                var vertexBuffer = mesh.GetVertexBuffer(0);
                var indexBuffer = mesh.GetIndexBuffer();
                tempBufferCache.Add(vertexBuffer);
                tempBufferCache.Add(indexBuffer);
                var baseTexture = pair.meshRenderer.sharedMaterial.mainTexture;
                var baseTextureOffset = pair.meshRenderer.sharedMaterial.mainTextureOffset;
                var baseTextureScale = pair.meshRenderer.sharedMaterial.mainTextureScale;

                // Determine where in the Unity vertex buffer each vertex attribute is
                var attributes = SceneUtils.FindAttributes(mesh);

                for (int i = 0; i < mesh.subMeshCount; i++)
                {
                    var submesh = mesh.GetSubMesh(i);
                    indexCount += submesh.indexCount;
                }

                // Configure material
                MaterialPropertyBlock mpb = null;
                if(l >= mpbs.Count)
                {
                    mpbs.Add(new MaterialPropertyBlock());
                }
                mpb = mpbs[l];
                mpb.Clear();
                mpb.SetBuffer(ShaderIds.VertexBuffer, vertexBuffer);
                mpb.SetBuffer(ShaderIds.IndexBuffer, indexBuffer);
                mpb.SetInt(ShaderIds.VertexStride, attributes.vertexStride);
                mpb.SetInt(ShaderIds.PositionOffset, attributes.position.offset);
                mpb.SetInt(ShaderIds.UVOffset, attributes.uv0.offset);
                mpb.SetMatrix(ShaderIds._ObjectToWorld, tr.localToWorldMatrix);
                cmd.SetKeyword(voxelMaterial, has32BitIndicesKeyword, (mesh.indexFormat == IndexFormat.UInt32));
                cmd.SetKeyword(voxelMaterial, hasUVsKeyword, attributes.uv0.exists);
                if(baseTexture != null)
                {
                    mpb.SetTexture(ShaderIds._BaseMap, baseTexture);
                    mpb.SetVector(ShaderIds._BaseMap_ST, new Vector4(baseTextureScale.x, baseTextureScale.y, baseTextureOffset.x, baseTextureOffset.y));
                }

                // Draw each triangle as an instance
                cmd.SetViewport(new Rect(0, 0, resolution, resolution));
                cmd.SetRandomWriteTarget(1, volume);
                cmd.DrawProcedural(tr.localToWorldMatrix, voxelMaterial, 0, MeshTopology.Triangles, indexCount, 1, mpb);
                l++;
            }

            Graphics.ExecuteCommandBuffer(cmd);
            Graphics.ClearRandomWriteTargets();
            CommandBufferPool.Release(cmd);

            foreach (var buffer in tempBufferCache)
            {
                buffer.Dispose();
            }
            tempBufferCache.Clear();

            
        }

        private void Update ()
        {
            if(debugDrawEveryFrame && volume != null && volume.IsCreated())
            {
                RenderAll();
            }
        }

        private void OnDrawGizmosSelected ()
        {
            Gizmos.color = Color.red;
            Gizmos.DrawWireCube(transform.position, Vector3.one * resolution * scale);
        }
    }

    [System.Serializable]
    public class MeshPair
    {
        public Mesh mesh;
        public MeshRenderer meshRenderer;
        public MeshPair (Mesh mesh, MeshRenderer meshRenderer)
        {
            this.mesh = mesh;
            this.meshRenderer = meshRenderer;
        }
    }
}
