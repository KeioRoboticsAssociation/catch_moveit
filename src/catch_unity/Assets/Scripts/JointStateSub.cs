using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

/// <summary>
/// ROSの /joint_states トピックを受信し、Unity 上のアームを動かすスクリプト
/// </summary>
public class JointStateSubscriber : MonoBehaviour
{
    [Header("ROS Settings")]
    [SerializeField] private string topicName = "/joint_states";
    
    [Header("Robot Settings")]
    [Tooltip("URDFでインポートしたロボットのルートオブジェクト")]
    [SerializeField] private GameObject robotRoot;

    // ROSとの通信
    private ROSConnection ros;
    
    // ROSのjoint名 → UnityのArticulationBody
    private Dictionary<string, ArticulationBody> jointNameToBody;
    
    // 受信データ（スレッドセーフ）
    private Dictionary<string, double> latestJointPositions;
    private readonly object lockObject = new object();

    void Start()
    {
        // 1. ROS接続
        ros = ROSConnection.GetOrCreateInstance();
        
        // 2. ArticulationBodyを名前でマッピング
        MapJoints();
        
        // 3. トピック購読開始
        ros.Subscribe<JointStateMsg>(topicName, OnJointStateReceived);
        
        Debug.Log($"JointStateSubscriber: Subscribed to {topicName}");
    }

    /// <summary>
    /// Robot Root 以下の ArticulationBody を名前でマッピング
    /// </summary>
    private void MapJoints()
    {
        jointNameToBody = new Dictionary<string, ArticulationBody>();
        
        if (robotRoot == null)
        {
            Debug.LogError("robotRoot is not assigned!");
            return;
        }

        ArticulationBody[] bodies = robotRoot.GetComponentsInChildren<ArticulationBody>(true);
        
        foreach (ArticulationBody body in bodies)
        {
            string jointName = body.name;
            
            if (jointNameToBody.ContainsKey(jointName))
            {
                Debug.LogWarning($"Duplicate joint name found: {jointName}");
            }
            else
            {
                jointNameToBody[jointName] = body;
                Debug.Log($"Mapped: {jointName} -> {body.name}");
            }
        }
        
        if (jointNameToBody.Count == 0)
        {
            Debug.LogError("No ArticulationBody found! Check robotRoot assignment.");
        }
    }

    /// <summary>
    /// ROSから /joint_states を受信
    /// </summary>
    private void OnJointStateReceived(JointStateMsg msg)
    {
        // データ整合性チェック
        if (msg.name == null || msg.position == null)
        {
            Debug.LogWarning("Received invalid JointState message");
            return;
        }

        if (msg.name.Length != msg.position.Length)
        {
            Debug.LogWarning("Joint names and positions count mismatch");
            return;
        }

        // 名前 → 角度 の辞書を作成（radian のまま）
        var newJointPositions = new Dictionary<string, double>();
        for (int i = 0; i < msg.name.Length; i++)
        {
            newJointPositions[msg.name[i]] = msg.position[i];
        }

        // スレッドセーフに更新
        lock (lockObject)
        {
            latestJointPositions = newJointPositions;
        }
    }

    /// <summary>
    /// UnityのUpdateでアームを動かす
    /// </summary>
    void Update()
    {
        // スレッドセーフに最新データを取得
        Dictionary<string, double> currentPositions;
        lock (lockObject)
        {
            currentPositions = latestJointPositions != null 
                ? new Dictionary<string, double>(latestJointPositions) 
                : null;
        }

        if (currentPositions == null || jointNameToBody == null) return;

        // 各jointを更新
        foreach (var kvp in currentPositions)
        {
            string jointName = kvp.Key;
            double targetAngleRad = kvp.Value;

            if (jointNameToBody.TryGetValue(jointName, out ArticulationBody body))
            {
                ArticulationDrive drive = body.xDrive;
                if (jointName == "left_Slider_1" || jointName == "left_Slider_2" || jointName == "right_Slider_1" || jointName == "right_Slider_2")
                {
                    drive.target = (float)(targetAngleRad / 0.024 * 0.026);
                }
                else if (jointName == "red_slider" || jointName == "blue_slider")
                {
                    drive.target = (float)targetAngleRad ;
                }
                else
                {
                    drive.target = (float)targetAngleRad * Mathf.Rad2Deg;
                }
                drive.forceLimit = 1000000f;
                body.xDrive = drive;
            }
        }
    }

    /// <summary>
    /// エディタ上でjoint名を確認するためのデバッグ表示
    /// </summary>
    private void OnValidate()
    {
        if (robotRoot != null)
        {
            var bodies = robotRoot.GetComponentsInChildren<ArticulationBody>(true);
            string jointNames = "";
            foreach (var body in bodies)
            {
                jointNames += body.name + ", ";
            }
            // Debug.Log("Available joints: " + jointNames);
        }
    }
}