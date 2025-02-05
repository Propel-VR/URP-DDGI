using UnityEngine;

public class CameraRenderingLayer : MonoBehaviour
{
    [SerializeField] RenderingLayerMask mask;
    [SerializeField] short canvasChannel;

    public RenderingLayerMask Mask => mask;

    public short CanvasChannel => canvasChannel;
}
