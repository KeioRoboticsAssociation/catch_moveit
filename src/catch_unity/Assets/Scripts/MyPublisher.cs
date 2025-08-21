using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class MyPublisher : MonoBehaviour
{
    ROSConnection ros;
    float time;

    // Start is called before the first frame update
    void Start()
    {
        // ROSコネクションの取得
        ros = ROSConnection.GetOrCreateInstance();

        // パブリッシャの登録
        ros.RegisterPublisher<StringMsg>("my_topic");
        
    }

    // Update is called once per frame
    void Update()
    {
        time += Time.deltaTime;
        if(time < 1.0f){
            return;
        }
        time = 0.0f;

        // メッセージのパブッシュ
        StringMsg msg = new StringMsg("Hello Unity!");
        ros.Publish("my_topic", msg);
    }
}
