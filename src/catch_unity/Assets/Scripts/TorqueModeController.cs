// TorqueModeController.cs
using UnityEngine;

[RequireComponent(typeof(ArticulationBody))]
public class TorqueModeController : MonoBehaviour
{
    public float stiffness = 1000f;   // Pゲイン（硬さ）
    public float damping = 300f;     // Dゲイン（減衰）
    // public float forceLimit = 500f;  // 最大トルク

    private ArticulationBody joint;
    private float initialAngle;

    void Start()
    {
        joint = GetComponent<ArticulationBody>();
        initialAngle = joint.jointPosition[0]; // 初期角度を記録
    }

    void FixedUpdate()
    {
        ArticulationDrive drive = joint.xDrive; // 回転軸に注意（xDriveなど）
        // drive.target = initialAngle;
        drive.stiffness = stiffness;
        drive.damping = damping;
        // drive.forceLimit = forceLimit;
        // joint.xDrive = drive;
    }
}