using System.Collections.Generic;
using UnityEngine;

public class GripperTriggerAttach : MonoBehaviour
{
    [Header("Target Settings")]
    [Tooltip("掴む対象オブジェクトのタグ（例: GrabbableObject）")]
    public string targetTag = "GrabbableObject";
    
    [Header("Parent Settings")]
    [Tooltip("掴んだオブジェクトの親にするオブジェクト")]
    public Transform parentTarget = null;

    // 現在掴んでいるオブジェクトのリスト
    private HashSet<GameObject> grabbedObjects = new HashSet<GameObject>();
    // 元の親を保存（復元用）
    private Dictionary<GameObject, Transform> originalParents = new Dictionary<GameObject, Transform>();
    // 元のRigidbody設定を保存
    private Dictionary<GameObject, bool> originalKinematic = new Dictionary<GameObject, bool>();
    private Dictionary<GameObject, bool> originalGravity = new Dictionary<GameObject, bool>();

    void Start()
    {
        // Triggerセットアップの診断
        Debug.Log($"=== Trigger Setup Diagnostic for {gameObject.name} ===");
        
        // Colliderの確認
        Collider col = GetComponent<Collider>();
        if (col == null)
        {
            Debug.LogError($"No Collider found on {gameObject.name}! Please add a Collider component.");
        }
        else
        {
            Debug.Log($"Collider found: {col.GetType().Name}");
            if (!col.isTrigger)
            {
                Debug.LogError($"Collider on {gameObject.name} is NOT set as Trigger! Please check 'Is Trigger' in the Collider component.");
            }
            else
            {
                Debug.Log($"Collider is properly set as Trigger");
            }
        }
        
        // Rigidbodyの確認
        Rigidbody rb = GetComponent<Rigidbody>();
        if (rb != null)
        {
            Debug.Log($"Rigidbody found: kinematic={rb.isKinematic}");
        }
        else
        {
            Debug.Log("No Rigidbody on this object (may be okay for static triggers)");
        }
        
        // 設定の確認
        Debug.Log($"Target tag: '{targetTag}'");
        Debug.Log($"Parent target: {(parentTarget != null ? parentTarget.name : "null (will use self)")}");
        
        Debug.Log("=== End Diagnostic ===");
    }

    void OnTriggerEnter(Collider other)
    {
        GameObject obj = other.gameObject;

        // デバッグ：侵入したオブジェクトの詳細情報
        Rigidbody rb = obj.GetComponent<Rigidbody>();
        Debug.Log($"[DEBUG] Trigger entered by: {obj.name}");
        Debug.Log($"[DEBUG] - Tag: '{obj.tag}' (expected: '{targetTag}')");
        Debug.Log($"[DEBUG] - Has Rigidbody: {rb != null}");
        if (rb != null)
        {
            Debug.Log($"[DEBUG] - Rigidbody isKinematic: {rb.isKinematic}");
        }
        Debug.Log($"[DEBUG] - Collider type: {other.GetType().Name}");
        Debug.Log($"[DEBUG] - Collider isTrigger: {other.isTrigger}");

        // 指定したタグを持つオブジェクトのみを対象とする
        if (!obj.CompareTag(targetTag))
        {
            Debug.Log($"[DEBUG] Tag mismatch - skipping {obj.name}");
            return; // タグが違う場合は何もしない
        }

        // デバッグ：対象タグのオブジェクトの侵入を記録
        Debug.Log($"✓ Target object entered trigger: {obj.name} (tag: {obj.tag})");

        // 既に掴んでいるオブジェクトは無視
        if (grabbedObjects.Contains(obj))
        {
            Debug.Log($"Object already grabbed: {obj.name}");
            return;
        }

        // ロボット部品は除外
        if (obj.transform.IsChildOf(transform.root))
        {
            Debug.Log($"Robot part excluded: {obj.name}");
            return;
        }

        Debug.Log($"Object entered gripper trigger: {obj.name}");
        AttachObject(obj);
    }

    void OnTriggerExit(Collider other)
    {
        GameObject obj = other.gameObject;
        
        // 対象タグのオブジェクトのみ処理
        if (!obj.CompareTag(targetTag))
        {
            return; // タグが違う場合は何もしない
        }
        
        // 掴んでいるオブジェクトが離れた場合は解放
        if (grabbedObjects.Contains(obj))
        {
            Debug.Log($"Target object exited trigger: {obj.name}");
            DetachObject(obj);
        }
    }

    private void AttachObject(GameObject obj)
    {
        if (obj == null || grabbedObjects.Contains(obj)) return;

        // デバッグ：アタッチ前の位置を記録
        Vector3 worldPosBefore = obj.transform.position;
        Vector3 worldRotBefore = obj.transform.eulerAngles;
        Debug.Log($"[ATTACH] Before - Position: {worldPosBefore}, Rotation: {worldRotBefore}");

        // 元の親を保存
        originalParents[obj] = obj.transform.parent;

        // Rigidbodyがある場合は設定を保存して調整
        Rigidbody rb = obj.GetComponent<Rigidbody>();
        if (rb != null)
        {
            originalKinematic[obj] = rb.isKinematic;
            originalGravity[obj] = rb.useGravity;
            
            // シンプルアプローチ：完全にkinematicにして物理演算を切る
            rb.isKinematic = true;   // 物理演算を完全無効化
            rb.useGravity = false;   // 重力も無効化
            rb.velocity = Vector3.zero;        // 速度をリセット
            rb.angularVelocity = Vector3.zero; // 角速度をリセット
            
            Debug.Log($"[ATTACH] Simple approach: Full kinematic mode");
        }

        // 親子化する前の世界座標を保持
        Vector3 worldPosition = obj.transform.position;
        Quaternion worldRotation = obj.transform.rotation;

        // 指定した親オブジェクトの子にする
        Transform newParent = (parentTarget != null) ? parentTarget : transform;
        obj.transform.SetParent(newParent, true); // trueで世界座標を維持
        
        // シンプルアプローチ：位置は親子関係に任せる
        // 複雑な座標計算はしない
        
        // 掴んだオブジェクトリストに追加
        grabbedObjects.Add(obj);

        // デバッグ：アタッチ後の位置を記録
        Vector3 worldPosAfter = obj.transform.position;
        Vector3 localPosAfter = obj.transform.localPosition;
        Debug.Log($"[ATTACH] After - World Position: {worldPosAfter}, Local Position: {localPosAfter}");
        Debug.Log($"[ATTACH] Parent: {newParent.name}, Parent Position: {newParent.position}");
        Debug.Log($"Object attached to parent: {obj.name} -> {newParent.name}");
    }

    private void DetachObject(GameObject obj)
    {
        if (obj == null || !grabbedObjects.Contains(obj)) return;

        // 元の親を復元
        if (originalParents.TryGetValue(obj, out Transform originalParent))
        {
            obj.transform.SetParent(originalParent);
            originalParents.Remove(obj);
        }

        // Rigidbodyの設定を復元
        Rigidbody rb = obj.GetComponent<Rigidbody>();
        if (rb != null)
        {
            if (originalKinematic.TryGetValue(obj, out bool wasKinematic))
            {
                rb.isKinematic = wasKinematic;
                originalKinematic.Remove(obj);
            }
            
            if (originalGravity.TryGetValue(obj, out bool hadGravity))
            {
                rb.useGravity = hadGravity;
                originalGravity.Remove(obj);
            }
            
            Debug.Log($"[DETACH] Settings restored: kinematic={rb.isKinematic}, gravity={rb.useGravity}");
        }

        // 掴んだオブジェクトリストから削除
        grabbedObjects.Remove(obj);

        Debug.Log($"Object detached: {obj.name}");
    }

    // 手動で全てのオブジェクトを解放
    public void ReleaseAllObjects()
    {
        var objectsToRelease = new List<GameObject>(grabbedObjects);
        foreach (var obj in objectsToRelease)
        {
            DetachObject(obj);
        }
    }

    // 現在掴んでいるオブジェクト数を取得
    public int GetAttachedObjectCount()
    {
        return grabbedObjects.Count;
    }

    void OnDestroy()
    {
        // スクリプト削除時に全て解放
        ReleaseAllObjects();
    }
}
