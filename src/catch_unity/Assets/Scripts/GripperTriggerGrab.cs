using System.Collections.Generic;
using UnityEngine;

public class GripperTriggerGrab : MonoBehaviour
{
    [Header("Target Settings")]
    [Tooltip("掴む対象オブジェクトのタグ（例: GrabbableObject）")]
    public string targetTag = "GrabbableObject";
    
    [Header("Parent Settings")]
    [Tooltip("掴んだオブジェクトの親にするオブジェクト（空の場合はこのオブジェクト）")]
    public Transform parentTarget = null;
    
    [Header("Gripper Settings")]
    [Tooltip("有効なグリッパー名のリスト")]
    public List<string> allowedGripperNames = new List<string>
    {
        "left_Slider_1", "left_Slider_2", "right_Slider_1", "right_Slider_2"
    };

    // 現在掴んでいるオブジェクトのリスト
    private HashSet<GameObject> grabbedObjects = new HashSet<GameObject>();
    // 元の親を保存（復元用）
    private Dictionary<GameObject, Transform> originalParents = new Dictionary<GameObject, Transform>();
    // 元のRigidbody設定を保存
    private Dictionary<GameObject, RigidbodySettings> originalRigidbodySettings = new Dictionary<GameObject, RigidbodySettings>();

    // Rigidbodyの設定を保存する構造体
    [System.Serializable]
    public struct RigidbodySettings
    {
        public bool isKinematic;
        public bool useGravity;
        public float mass;
        public float drag;
        public float angularDrag;
    }

    void Start()
    {
        // parentTargetが指定されていない場合は自分自身を使用
        if (parentTarget == null)
        {
            parentTarget = transform;
        }

        // このオブジェクトにTriggerコライダーがあることを確認
        Collider col = GetComponent<Collider>();
        if (col == null)
        {
            Debug.LogWarning($"{gameObject.name}: Collider component not found. Please add a Collider and set isTrigger = true.");
        }
        else if (!col.isTrigger)
        {
            Debug.LogWarning($"{gameObject.name}: Collider should be set as Trigger for proper functionality.");
        }
    }

    void OnTriggerEnter(Collider other)
    {
        GameObject obj = other.gameObject;
        
        // 指定されたグリッパー名のチェック（空の場合はスキップ）
        if (allowedGripperNames.Count > 0 && !allowedGripperNames.Contains(gameObject.name))
        {
            return;
        }

        // 指定したタグを持つオブジェクトのみを対象とする
        if (!obj.CompareTag(targetTag))
        {
            Debug.Log($"Ignored trigger with non-target tag: {gameObject.name} <-> {obj.name} (tag: {obj.tag}, expected: {targetTag})");
            return;
        }

        // 既に掴んでいるオブジェクトは無視
        if (grabbedObjects.Contains(obj))
        {
            return;
        }

        // ロボット部品同士の衝突を除外
        if (obj.transform.IsChildOf(transform.root))
        {
            Debug.Log($"Ignored robot part trigger: {gameObject.name} <-> {obj.name} (Same robot)");
            return;
        }

        Debug.Log($"Target entered trigger: gripper:{gameObject.name} -> object:{obj.name} (tag:{obj.tag})");
        
        GrabObject(obj);
    }

    void OnTriggerExit(Collider other)
    {
        GameObject obj = other.gameObject;
        
        // 掴んでいるオブジェクトが離れた場合は解放
        if (grabbedObjects.Contains(obj))
        {
            Debug.Log($"Target exited trigger: {gameObject.name} -> {obj.name}");
            ReleaseObject(obj);
        }
    }

    private void GrabObject(GameObject obj)
    {
        if (obj == null || grabbedObjects.Contains(obj)) return;

        // 元の親を保存
        originalParents[obj] = obj.transform.parent;

        // Rigidbodyの元の設定を保存
        Rigidbody rb = obj.GetComponent<Rigidbody>();
        if (rb != null)
        {
            originalRigidbodySettings[obj] = new RigidbodySettings
            {
                isKinematic = rb.isKinematic,
                useGravity = rb.useGravity,
                mass = rb.mass,
                drag = rb.drag,
                angularDrag = rb.angularDrag
            };

            // Rigidbodyを親子化に適した設定に変更
            rb.isKinematic = true;  // 物理演算を無効化（親の動きに完全追従）
            rb.useGravity = false;  // 重力を無効化
        }
        else
        {
            // Rigidbodyがない場合は追加してkinematicに設定
            rb = obj.AddComponent<Rigidbody>();
            rb.isKinematic = true;
            rb.useGravity = false;
            
            // デフォルト設定を保存（後で削除するため）
            originalRigidbodySettings[obj] = new RigidbodySettings
            {
                isKinematic = false,
                useGravity = true,
                mass = 1f,
                drag = 0f,
                angularDrag = 0.05f
            };
        }

        // 指定した親オブジェクトの子にする
        obj.transform.SetParent(parentTarget);
        
        // 掴んだオブジェクトリストに追加
        grabbedObjects.Add(obj);

        Debug.Log($"Object grabbed and parented: {obj.name} -> {parentTarget.name} (kinematic: {rb.isKinematic})");
    }

    private void ReleaseObject(GameObject obj)
    {
        if (obj == null || !grabbedObjects.Contains(obj)) return;

        // 元の親を復元
        if (originalParents.TryGetValue(obj, out Transform originalParent))
        {
            obj.transform.SetParent(originalParent);
            originalParents.Remove(obj);
        }

        // Rigidbodyの設定を復元
        if (originalRigidbodySettings.TryGetValue(obj, out RigidbodySettings originalSettings))
        {
            Rigidbody rb = obj.GetComponent<Rigidbody>();
            if (rb != null)
            {
                rb.isKinematic = originalSettings.isKinematic;
                rb.useGravity = originalSettings.useGravity;
                rb.mass = originalSettings.mass;
                rb.drag = originalSettings.drag;
                rb.angularDrag = originalSettings.angularDrag;
            }
            originalRigidbodySettings.Remove(obj);
        }

        // 掴んだオブジェクトリストから削除
        grabbedObjects.Remove(obj);

        Debug.Log($"Object released: {obj.name} (restored to original parent and settings)");
    }

    // 手動でオブジェクトを解放する公開メソッド
    public void ReleaseAllObjects()
    {
        var objectsToRelease = new List<GameObject>(grabbedObjects);
        foreach (var obj in objectsToRelease)
        {
            ReleaseObject(obj);
        }
        Debug.Log($"All objects released by {gameObject.name}");
    }

    // 現在掴んでいるオブジェクト数を取得
    public int GetGrabbedObjectCount()
    {
        return grabbedObjects.Count;
    }

    // 特定のオブジェクトが掴まれているかチェック
    public bool IsObjectGrabbed(GameObject obj)
    {
        return grabbedObjects.Contains(obj);
    }

    // デバッグ用：掴んでいるオブジェクトの一覧を表示
    [System.Diagnostics.Conditional("UNITY_EDITOR")]
    public void DebugGrabbedObjects()
    {
        Debug.Log($"Grabbed objects by {gameObject.name}:");
        foreach (var obj in grabbedObjects)
        {
            Debug.Log($"  - {obj.name}");
        }
    }

    void OnDestroy()
    {
        // オブジェクト削除時に全て解放
        ReleaseAllObjects();
    }
}
