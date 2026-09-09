using UnityEngine;

public class URPMaterialOpacityController : MonoBehaviour
{
    [Tooltip("If left empty, the script will automatically find all Renderers on this GameObject and its children.")]
    public Renderer[] targetRenderers;

    private void Awake()
    {
        // If no renderers were manually assigned, find all of them on this object and its children
        if (targetRenderers == null || targetRenderers.Length == 0)
        {
            targetRenderers = GetComponentsInChildren<Renderer>();
        }
    }

    /// <summary>
    /// Sets the opacity of all materials on the target renderers.
    /// </summary>
    /// <param name="opacity">A value between 0.0 and 1.0 representing the desired opacity.</param>
    public void SetOpacity(float opacity)
    {
        if (targetRenderers == null || targetRenderers.Length == 0)
        {
            Debug.LogWarning("No target renderers found for URPMaterialOpacityController on " + gameObject.name, this);
            return;
        }

        // Clamp the value to ensure it stays between 0 and 1
        opacity = Mathf.Clamp01(opacity);

        foreach (Renderer rend in targetRenderers)
        {
            if (rend == null) continue;

            foreach (Material mat in rend.materials)
            {
                if (mat.HasProperty("_BaseColor"))
                {
                    Color color = mat.GetColor("_BaseColor");
                    color.a = opacity;
                    mat.SetColor("_BaseColor", color);
                }
                else if (mat.HasProperty("_Color")) // Fallback for standard shader
                {
                    Color color = mat.GetColor("_Color");
                    color.a = opacity;
                    mat.SetColor("_Color", color);
                }
            }
        }
    }
}
