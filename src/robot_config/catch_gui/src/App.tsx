import React, { useState, useEffect, useRef } from 'react';
import ROSLIB from 'roslib';
import './App.css';

// --- ROS 2 接続設定 ---
const ROSBRIDGE_SERVER_URL = "ws://192.168.1.7:9090";
const COMMAND_TOPIC_NAME = "/robot_command";
const COMMAND_MESSAGE_TYPE = "std_msgs/msg/String";
const POSE_TOPIC_NAME = "/left_target_pose_rpy";
const POSE_MESSAGE_TYPE = "std_msgs/msg/Float64MultiArray";
const ARM1_UP_TOPIC = "/left_arm_up";        // アーム1用upトピック
const ARM1_DOWN_TOPIC = "/left_arm_down";    // アーム1用downトピック
const ARM2_UP_TOPIC = "/right_arm_up";        // アーム2用upトピック
const ARM2_DOWN_TOPIC = "/right_arm_down";    // アーム2用downトピック
const ARM1_GRAB_TOPIC = "/left_arm_close";    // アーム1掴むトピック
const ARM1_RELEASE_TOPIC = "/left_arm_open";  // アーム1離すトピック
const ARM2_GRAB_TOPIC = "/right_arm_close";   // アーム2掴むトピック
const ARM2_RELEASE_TOPIC = "/right_arm_open"; // アーム2離すトピック
const UP_DOWN_MESSAGE_TYPE = "std_msgs/msg/String";

export default function App() {
  const [connectionStatus, setConnectionStatus] = useState("Disconnected");
  const [lastReceivedMessage, setLastReceivedMessage] = useState("");
  const [backgroundColor, setBackgroundColor] = useState("red");
  const [posePublisher, setPosePublisher] = useState(null);
  const [arm1UpPublisher, setArm1UpPublisher] = useState(null);    // アーム1用upパブリッシャー
  const [arm1DownPublisher, setArm1DownPublisher] = useState(null); // アーム1用downパブリッシャー
  const [arm2UpPublisher, setArm2UpPublisher] = useState(null);    // アーム2用upパブリッシャー
  const [arm2DownPublisher, setArm2DownPublisher] = useState(null); // アーム2用downパブリッシャー
  const [arm1GrabPublisher, setArm1GrabPublisher] = useState(null);    // アーム1掴むパブリッシャー
  const [arm1ReleasePublisher, setArm1ReleasePublisher] = useState(null); // アーム1離すパブリッシャー
  const [arm2GrabPublisher, setArm2GrabPublisher] = useState(null);    // アーム2掴むパブリッシャー
  const [arm2ReleasePublisher, setArm2ReleasePublisher] = useState(null); // アーム2離すパブリッシャー

  const ros = useRef(null);
  const publisher = useRef(null);
  const listener = useRef(null);

  useEffect(() => {
    ros.current = new ROSLIB.Ros({
      url: ROSBRIDGE_SERVER_URL
    });

    ros.current.on('connection', () => {
      console.log('Connected to websocket server.');
      setConnectionStatus("Connected");
      initializePublisher();
      initializeSubscriber();
      initializePosePublisher();
      initializeUpDownPublishers(); // up/downパブリッシャー初期化
      initializeGrabReleasePublishers(); // 掴む/離すパブリッシャー初期化
    });

    ros.current.on('error', (error) => {
      console.log('Error connecting to websocket server: ', error);
      setConnectionStatus("Error");
    });

    ros.current.on('close', () => {
      console.log('Connection to websocket server closed.');
      setConnectionStatus("Disconnected");
    });

    return () => {
      if (ros.current && ros.current.isConnected) {
        if (listener.current) {
          listener.current.unsubscribe();
        }
        ros.current.close();
      }
    };
  }, []);

  const initializePublisher = () => {
    publisher.current = new ROSLIB.Topic({
      ros: ros.current,
      name: COMMAND_TOPIC_NAME,
      messageType: COMMAND_MESSAGE_TYPE
    });
  };

  const initializePosePublisher = () => {
    const posePub = new ROSLIB.Topic({
      ros: ros.current,
      name: POSE_TOPIC_NAME,
      messageType: POSE_MESSAGE_TYPE
    });
    setPosePublisher(posePub);
  };

  // up/downパブリッシャーの初期化
  const initializeUpDownPublishers = () => {
    // アーム1用
    const arm1UpPub = new ROSLIB.Topic({
      ros: ros.current,
      name: ARM1_UP_TOPIC,
      messageType: UP_DOWN_MESSAGE_TYPE
    });
    setArm1UpPublisher(arm1UpPub);

    const arm1DownPub = new ROSLIB.Topic({
      ros: ros.current,
      name: ARM1_DOWN_TOPIC,
      messageType: UP_DOWN_MESSAGE_TYPE
    });
    setArm1DownPublisher(arm1DownPub);

    // アーム2用
    const arm2UpPub = new ROSLIB.Topic({
      ros: ros.current,
      name: ARM2_UP_TOPIC,
      messageType: UP_DOWN_MESSAGE_TYPE
    });
    setArm2UpPublisher(arm2UpPub);

    const arm2DownPub = new ROSLIB.Topic({
      ros: ros.current,
      name: ARM2_DOWN_TOPIC,
      messageType: UP_DOWN_MESSAGE_TYPE
    });
    setArm2DownPublisher(arm2DownPub);
  };

  // 掴む/離すパブリッシャーの初期化
  const initializeGrabReleasePublishers = () => {
    // アーム1用
    const arm1GrabPub = new ROSLIB.Topic({
      ros: ros.current,
      name: ARM1_GRAB_TOPIC,
      messageType: UP_DOWN_MESSAGE_TYPE
    });
    setArm1GrabPublisher(arm1GrabPub);

    const arm1ReleasePub = new ROSLIB.Topic({
      ros: ros.current,
      name: ARM1_RELEASE_TOPIC,
      messageType: UP_DOWN_MESSAGE_TYPE
    });
    setArm1ReleasePublisher(arm1ReleasePub);

    // アーム2用
    const arm2GrabPub = new ROSLIB.Topic({
      ros: ros.current,
      name: ARM2_GRAB_TOPIC,
      messageType: UP_DOWN_MESSAGE_TYPE
    });
    setArm2GrabPublisher(arm2GrabPub);

    const arm2ReleasePub = new ROSLIB.Topic({
      ros: ros.current,
      name: ARM2_RELEASE_TOPIC,
      messageType: UP_DOWN_MESSAGE_TYPE
    });
    setArm2ReleasePublisher(arm2ReleasePub);
  };

  const initializeSubscriber = () => {
    listener.current = new ROSLIB.Topic({
      ros: ros.current,
      name: COMMAND_TOPIC_NAME,
      messageType: COMMAND_MESSAGE_TYPE
    });

    listener.current.subscribe((message) => {
      setLastReceivedMessage(message.data);
    });
  };

  // 各ボタンに対応するPose値の定義
  const buttonPoseValues = {
    1: [1.0, 2.0, 3.0, 0.0, 0.0, 0.0],
    2: [1.5, 2.5, 3.5, 0.1, 0.1, 0.1],
    3: [2.0, 3.0, 4.0, 0.2, 0.2, 0.2],
    4: [2.5, 3.5, 4.5, 0.3, 0.3, 0.3],
    5: [3.0, 4.0, 5.0, 0.4, 0.4, 0.4],
    6: [3.5, 4.5, 5.5, 0.5, 0.5, 0.5],
    7: [4.0, 5.0, 6.0, 0.6, 0.6, 0.6],
    8: [4.5, 5.5, 6.5, 0.7, 0.7, 0.7],
    9: [5.0, 6.0, 7.0, 0.8, 0.8, 0.8],
    10: [5.5, 6.5, 7.5, 0.9, 0.9, 0.9],
    11: [6.0, 7.0, 8.0, 1.0, 1.0, 1.0],
    12: [6.5, 7.5, 8.5, 1.1, 1.1, 1.1],
    13: [7.0, 8.0, 9.0, 1.2, 1.2, 1.2],
    14: [7.5, 8.5, 9.5, 1.3, 1.3, 1.3],
    15: [8.0, 9.0, 10.0, 1.4, 1.4, 1.4],
    16: [0.282, 0.3705, 0.2, 0.0, 0.0, 1.57],
    17: [0.282, 0.3705, 0.086, 0.0, 0.0, 1.57],
    18: [9.5, 10.5, 11.5, 1.7, 1.7, 1.7],
    19: [10.0, 11.0, 12.0, 1.8, 1.8, 1.8],
    20: [10.5, 11.5, 12.5, 1.9, 1.9, 1.9]
  };

  // アーム1の初期位置とゴール位置
  const arm1Positions = {
    initial: [-0.534, 0.00, 0.445, 0.0, 0, 3.1415],
    goal: [-0.320, -0.7345, 0.186, 0.0, 0.0, 0.0]
  };

  // アーム2の初期位置とゴール位置
  const arm2Positions = {
    initial: [2.0, 2.0, 2.0, 0.0, 0.0, 0.0],
    goal: [3.0, 3.0, 3.0, 0.5, 0.5, 0.5]
  };

  const handleButtonClick = (commandText) => {
    if (publisher.current && connectionStatus === 'Connected') {
      const message = new ROSLIB.Message({
        data: commandText
      });
      publisher.current.publish(message);
      console.log(`🚀 Published to ${COMMAND_TOPIC_NAME}: "${commandText}"`);
    } else {
      console.warn(`Cannot send command. ROS Status: ${connectionStatus}`);
    }
  };

  // Pose値をpublishする関数
  const handlePoseButtonClick = (buttonNumber) => {
    if (posePublisher && connectionStatus === 'Connected') {
      const poseValues = buttonPoseValues[buttonNumber] || [0, 0, 0, 0, 0, 0];
      const message = new ROSLIB.Message({
        data: poseValues
      });
      posePublisher.publish(message);
      console.log(`🎯 Published pose to ${POSE_TOPIC_NAME}: [${poseValues.join(', ')}]`);
    } else {
      console.warn(`Cannot send pose. ROS Status: ${connectionStatus}`);
    }
  };

  // アーム1のupボタンをpublishする関数
  const handleArm1UpButtonClick = () => {
    if (arm1UpPublisher && connectionStatus === 'Connected') {
      const message = new ROSLIB.Message({
        data: "up"
      });
      arm1UpPublisher.publish(message);
      console.log(`⬆️ Published to ${ARM1_UP_TOPIC}: "up"`);
    } else {
      console.warn(`Cannot send arm1 up command. ROS Status: ${connectionStatus}`);
    }
  };

  // アーム1のdownボタンをpublishする関数
  const handleArm1DownButtonClick = () => {
    if (arm1DownPublisher && connectionStatus === 'Connected') {
      const message = new ROSLIB.Message({
        data: "down"
      });
      arm1DownPublisher.publish(message);
      console.log(`⬇️ Published to ${ARM1_DOWN_TOPIC}: "down"`);
    } else {
      console.warn(`Cannot send arm1 down command. ROS Status: ${connectionStatus}`);
    }
  };

  // アーム2のupボタンをpublishする関数
  const handleArm2UpButtonClick = () => {
    if (arm2UpPublisher && connectionStatus === 'Connected') {
      const message = new ROSLIB.Message({
        data: "up"
      });
      arm2UpPublisher.publish(message);
      console.log(`⬆️ Published to ${ARM2_UP_TOPIC}: "up"`);
    } else {
      console.warn(`Cannot send arm2 up command. ROS Status: ${connectionStatus}`);
    }
  };

  // アーム2のdownボタンをpublishする関数
  const handleArm2DownButtonClick = () => {
    if (arm2DownPublisher && connectionStatus === 'Connected') {
      const message = new ROSLIB.Message({
        data: "down"
      });
      arm2DownPublisher.publish(message);
      console.log(`⬇️ Published to ${ARM2_DOWN_TOPIC}: "down"`);
    } else {
      console.warn(`Cannot send arm2 down command. ROS Status: ${connectionStatus}`);
    }
  };

  // アーム1の掴むボタンをpublishする関数
  const handleArm1Grab = () => {
    if (arm1GrabPublisher && connectionStatus === 'Connected') {
      const message = new ROSLIB.Message({
        data: "close"
      });
      arm1GrabPublisher.publish(message);
      console.log(`✋ Published to ${ARM1_GRAB_TOPIC}: "close"`);
    } else {
      console.warn(`Cannot send arm1 grab command. ROS Status: ${connectionStatus}`);
    }
    // handleButtonClick("アーム1 抓む"); // 元のコマンドは削除
  };

  // アーム1の離すボタンをpublishする関数
  const handleArm1Release = () => {
    if (arm1ReleasePublisher && connectionStatus === 'Connected') {
      const message = new ROSLIB.Message({
        data: "open"
      });
      arm1ReleasePublisher.publish(message);
      console.log(`👐 Published to ${ARM1_RELEASE_TOPIC}: "open"`);
    } else {
      console.warn(`Cannot send arm1 release command. ROS Status: ${connectionStatus}`);
    }
    // handleButtonClick("アーム1 離す"); // 元のコマンドは削除
  };

  // アーム2の掴むボタンをpublishする関数
  const handleArm2Grab = () => {
    if (arm2GrabPublisher && connectionStatus === 'Connected') {
      const message = new ROSLIB.Message({
        data: "close"
      });
      arm2GrabPublisher.publish(message);
      console.log(`✋ Published to ${ARM2_GRAB_TOPIC}: "close"`);
    } else {
      console.warn(`Cannot send arm2 grab command. ROS Status: ${connectionStatus}`);
    }
    // handleButtonClick("アーム2 抓む"); // 元のコマンドは削除
  };

  // アーム2の離すボタンをpublishする関数
  const handleArm2Release = () => {
    if (arm2ReleasePublisher && connectionStatus === 'Connected') {
      const message = new ROSLIB.Message({
        data: "open"
      });
      arm2ReleasePublisher.publish(message);
      console.log(`👐 Published to ${ARM2_RELEASE_TOPIC}: "open"`);
    } else {
      console.warn(`Cannot send arm2 release command. ROS Status: ${connectionStatus}`);
    }
    // handleButtonClick("アーム2 離す"); // 元のコマンドは削除
  };

  // アーム1の初期位置をpublish
  const handleArm1Initial = () => {
    if (posePublisher && connectionStatus === 'Connected') {
      const poseValues = arm1Positions.initial;
      const message = new ROSLIB.Message({
        data: poseValues
      });
      posePublisher.publish(message);
      console.log(`🎯 Published Arm1 Initial pose to ${POSE_TOPIC_NAME}: [${poseValues.join(', ')}]`);
    } else {
      console.warn(`Cannot send pose. ROS Status: ${connectionStatus}`);
    }
  };

  // アーム1のゴール位置をpublish
  const handleArm1Goal = () => {
    if (posePublisher && connectionStatus === 'Connected') {
      const poseValues = arm1Positions.goal;
      const message = new ROSLIB.Message({
        data: poseValues
      });
      posePublisher.publish(message);
      console.log(`🎯 Published Arm1 Goal pose to ${POSE_TOPIC_NAME}: [${poseValues.join(', ')}]`);
    } else {
      console.warn(`Cannot send pose. ROS Status: ${connectionStatus}`);
    }
  };

  // アーム2の初期位置をpublish
  const handleArm2Initial = () => {
    if (posePublisher && connectionStatus === 'Connected') {
      const poseValues = arm2Positions.initial;
      const message = new ROSLIB.Message({
        data: poseValues
      });
      posePublisher.publish(message);
      console.log(`🎯 Published Arm2 Initial pose to ${POSE_TOPIC_NAME}: [${poseValues.join(', ')}]`);
    } else {
      console.warn(`Cannot send pose. ROS Status: ${connectionStatus}`);
    }
  };

  // アーム2のゴール位置をpublish
  const handleArm2Goal = () => {
    if (posePublisher && connectionStatus === 'Connected') {
      const poseValues = arm2Positions.goal;
      const message = new ROSLIB.Message({
        data: poseValues
      });
      posePublisher.publish(message);
      console.log(`🎯 Published Arm2 Goal pose to ${POSE_TOPIC_NAME}: [${poseValues.join(', ')}]`);
    } else {
      console.warn(`Cannot send pose. ROS Status: ${connectionStatus}`);
    }
  };

  const toggleBackgroundColor = () => {
    setBackgroundColor(prevColor => prevColor === "red" ? "blue" : "red");
  };

  const GridButton = ({ buttonNumber }) => (
    <button 
      className="grid-button"
      onClick={() => handlePoseButtonClick(buttonNumber)}
      disabled={connectionStatus !== 'Connected'}
    >
      {`Pose ${buttonNumber}`}
    </button>
  );

  return (
    <div 
      className="app-container"
      style={{ backgroundColor: backgroundColor === "red" ? "#fee2e2" : "#dbeafe" }}
    >
      
      {/* ヘッダー */}
      <header className="app-header">
        <h1 className="app-title">Custom Robot Controller</h1>
        <div className="header-controls">
          <p className="status-text">
            Status: 
            <span className={`status-indicator ${
              connectionStatus === 'Connected' ? 'status-connected' :
              connectionStatus === 'Error' ? 'status-error' :
              'status-disconnected'
            }`}>
              {connectionStatus}
            </span>
          </p>
          <button 
            className="toggle-button"
            onClick={toggleBackgroundColor}
          >
            field: {backgroundColor === "red" ? "赤" : "青"}
          </button>
        </div>
      </header>
      
      {/* 4行5列のグリッド + up/downボタン */}
      <div className="button-grid-container">
        <div className="button-grid">
          {Array.from({ length: 20 }).map((_, index) => (
            <GridButton 
              key={index + 1} 
              buttonNumber={index + 1}
            />
          ))}
        </div>
        
        {/* アーム1のup/downボタン */}
        <div className="up-down-column">
          <div className="up-down-group">
            <h3 className="up-down-title">アーム1</h3>
            <button 
              className="up-down-button up-button"
              onClick={handleArm1UpButtonClick}
              disabled={connectionStatus !== 'Connected'}
            >
              ⬆️ UP
            </button>
            <button 
              className="up-down-button down-button"
              onClick={handleArm1DownButtonClick}
              disabled={connectionStatus !== 'Connected'}
            >
              ⬇️ DOWN
            </button>
          </div>
          
          {/* アーム2のup/downボタン */}
          <div className="up-down-group">
            <h3 className="up-down-title">アーム2</h3>
            <button 
              className="up-down-button up-button"
              onClick={handleArm2UpButtonClick}
              disabled={connectionStatus !== 'Connected'}
            >
              ⬆️ UP
            </button>
            <button 
              className="up-down-button down-button"
              onClick={handleArm2DownButtonClick}
              disabled={connectionStatus !== 'Connected'}
            >
              ⬇️ DOWN
            </button>
          </div>
        </div>
      </div>

      {/* Subscriber表示エリア */}
      <div className="subscriber-area">
        <span className="subscriber-label">Last Published Command:</span>
        <span className="subscriber-value">
          {lastReceivedMessage || "---"}
        </span>
      </div>

      {/* 下部コントロール */}
      <div className="control-area">
        {/* アーム1のコントロールボタン群 */}
        <div className="arm-controls">
          {/* アーム1の初期位置ボタン */}
          <button 
            className="arm-button"
            onClick={handleArm1Initial}
            disabled={connectionStatus !== 'Connected'}
          >
            アーム1<br/>初期位置
          </button>
          
          {/* アーム1のゴールボタン */}
          <button 
            className="arm-button"
            onClick={handleArm1Goal}
            disabled={connectionStatus !== 'Connected'}
          >
            アーム1<br/>ゴール
          </button>
          
          {/* アーム1の掴むボタン */}
          <button 
            className="arm-button grab-button"
            onClick={handleArm1Grab}
            disabled={connectionStatus !== 'Connected'}
          >
            アーム1<br/>掴む
          </button>
          
          {/* アーム1の離すボタン */}
          <button 
            className="arm-button release-button"
            onClick={handleArm1Release}
            disabled={connectionStatus !== 'Connected'}
          >
            アーム1<br/>離す
          </button>
        </div>
        
        {/* アーム2のコントロールボタン群 */}
        <div className="arm-controls">
          {/* アーム2の初期位置ボタン */}
          <button 
            className="arm-button"
            onClick={handleArm2Initial}
            disabled={connectionStatus !== 'Connected'}
          >
            アーム2<br/>初期位置
          </button>
          
          {/* アーム2のゴールボタン */}
          <button 
            className="arm-button"
            onClick={handleArm2Goal}
            disabled={connectionStatus !== 'Connected'}
          >
            アーム2<br/>ゴール
          </button>
          
          {/* アーム2の掴むボタン */}
          <button 
            className="arm-button grab-button"
            onClick={handleArm2Grab}
            disabled={connectionStatus !== 'Connected'}
          >
            アーム2<br/>掴む
          </button>
          
          {/* アーム2の離すボタン */}
          <button 
            className="arm-button release-button"
            onClick={handleArm2Release}
            disabled={connectionStatus !== 'Connected'}
          >
            アーム2<br/>離す
          </button>
        </div>
      </div>
    </div>
  );
}