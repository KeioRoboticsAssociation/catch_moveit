using System.Collections.Generic;
using UnityEngine;

public class GripperAddForce : MonoBehaviour
{
    [Header("Target Settings")]
    [Tooltip("掴む対象オブジェクトのタグ（例: GrabbableObject）")]
    public string targetTag = "GrabbableObject";
    
    [Header("ArticulationBody Joint Settings")]
    [Tooltip("ArticulationBodyで固定する対象オブジェクト（空の場合はグリッパ自身）")]
    public GameObject articulationJointTarget = null;

    // 衝突候補リスト（複数オブジェクトがぶつかる可能性に対応）
    private HashSet<GameObject> candidates = new HashSet<GameObject>();

    // 現在掴んでいるオブジェクト
    private GameObject grabbedObject = null;
    // 掴んだオブジェクトの Rigidbody
    private Rigidbody grabbedRb = null;
    // FixedJoint（固定に使用）
    private FixedJoint fixedJoint = null;

    void OnCollisionEnter(Collision collision)
    {
        GameObject obj = collision.gameObject;
        // Rigidbody を持つオブジェクトのみ候補とする
        Rigidbody rb = collision.rigidbody ?? obj.GetComponent<Rigidbody>();
        
        // 指定されたスライダ部位のみ掴み処理を行う
        var allowed = new HashSet<string>{
            "left_Slider_1", "left_Slider_2", "right_Slider_1", "right_Slider_2"
        };
        if (!allowed.Contains(gameObject.name)) return;

        // 指定したタグを持つオブジェクトのみを対象とする
        if (!obj.CompareTag(targetTag))
        {
            // Debug.Log($"Ignored collision with non-target tag: {gameObject.name} <-> {obj.name} (tag: {obj.tag}, expected: {targetTag})");
            return;
        }

        // 相手がArticulationBodyを持つ場合も除外（ロボット部品同士の衝突）
        if (obj.GetComponent<ArticulationBody>() != null)
        {
            // Debug.Log($"Ignored robot part collision: {gameObject.name} <-> {obj.name} (ArticulationBody)");
            return;
        }

        Debug.Log($"Target object collision: gripper:{gameObject.name} -> object:{obj.name} (tag:{obj.tag}, path:{GetHierarchyPath(obj.transform)})");
        
        if (rb != null)
        {
            candidates.Add(obj);
            // 衝突時にまだ掴んでいなければ自動で掴む
            if (grabbedObject == null)
            {
                Grab(obj);
            }
        }
    }

    void OnCollisionExit(Collision collision)
    {
        // GameObject obj = collision.gameObject;
        // if (candidates.Contains(obj))
        // {
        //     candidates.Remove(obj);
        // }
        // もし離れたオブジェクトが現在掴んでいるものであれば離す
        // if (grabbedObject == obj)
        // {
        //     Release();
        // }
    }

    private void Grab(GameObject obj)
    {
        if (obj == null) return;

        grabbedObject = obj;
        grabbedRb = obj.GetComponent<Rigidbody>();
        
        // Rigidbodyがない場合は追加
        if (grabbedRb == null)
        {
            grabbedRb = obj.AddComponent<Rigidbody>();
        }

        // FixedJointの接続先を決定（指定されていればarticulationJointTarget、なければグリッパ自身）
        GameObject jointTarget = articulationJointTarget != null ? articulationJointTarget : gameObject;
        
        // jointTargetのnullチェック
        if (jointTarget == null)
        {
            Debug.LogError("jointTarget is null in Grab method");
            return;
        }

        ArticulationBody targetRb = jointTarget.GetComponent<ArticulationBody>();

        // 接続先にRigidbodyがない場合は追加
        if (targetRb == null)
        {
            targetRb = jointTarget.AddComponent<ArticulationBody>();
            
            // AddComponentが成功したかチェック
            if (targetRb == null)
            {
                Debug.LogError($"Failed to add ArticulationBody to {jointTarget.name}");
                return;
            }
            
            // targetRb.isKinematic = true; // グリッパは動かない
        }
        
        // FixedJointを追加
        fixedJoint = obj.AddComponent<FixedJoint>();
        
        // FixedJointが正常に追加されたかチェック
        if (fixedJoint == null)
        {
            Debug.LogError($"Failed to add FixedJoint to {obj.name}");
            return;
        }
        
        fixedJoint.connectedArticulationBody = targetRb;
        fixedJoint.breakForce = Mathf.Infinity; // 無限の力まで耐える
        fixedJoint.breakTorque = Mathf.Infinity; // 無限のトルクまで耐える
        
        Debug.Log($"FixedJoint created: {obj.name} <-> {jointTarget.name}");
    }

    // 階層パスを取得（デバッグ用）
    private string GetHierarchyPath(Transform t)
    {
        if (t == null) return "";
        string path = t.name;
        while (t.parent != null)
        {
            t = t.parent;
            path = t.name + "/" + path;
        }
        return path;
    }

    private void Release()
    {
        if (grabbedObject == null) return;

        // FixedJointを破棄
        if (fixedJoint != null)
        {
            Destroy(fixedJoint); // DestroyImmediateではなくDestroyを使用
            fixedJoint = null;
            Debug.Log($"FixedJoint destroyed for {grabbedObject.name}");
        }

        Debug.Log($"Released by {gameObject.name}: {grabbedObject.name}");

        grabbedObject = null;
        grabbedRb = null;
    }
}
