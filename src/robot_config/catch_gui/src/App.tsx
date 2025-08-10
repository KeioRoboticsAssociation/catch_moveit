import React, { useState, useEffect, useRef } from 'react';
import ROSLIB from 'roslib';
import './App.css';

// --- ROS 2 接続設定 ---
const ROSBRIDGE_SERVER_URL = "ws://10.10.10.84:9090";
const COMMAND_TOPIC_NAME = "/robot_command";
const COMMAND_MESSAGE_TYPE = "std_msgs/msg/String";
const POSE_TOPIC_NAME = "/target_pose_rpy";
const POSE_MESSAGE_TYPE = "std_msgs/msg/Float64MultiArray";

export default function App() {
  const [connectionStatus, setConnectionStatus] = useState("Disconnected");
  const [lastReceivedMessage, setLastReceivedMessage] = useState("");
  const [backgroundColor, setBackgroundColor] = useState("red");
  const [posePublisher, setPosePublisher] = useState(null);

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
    16: [0.235, 0.331,0.098 , 0.041, 1.57, 0.041],
    17: [9.0, 10.0, 11.0, 1.6, 1.6, 1.6],
    18: [9.5, 10.5, 11.5, 1.7, 1.7, 1.7],
    19: [10.0, 11.0, 12.0, 1.8, 1.8, 1.8],
    20: [10.5, 11.5, 12.5, 1.9, 1.9, 1.9]
  };

  // アーム1の初期位置とゴール位置
  const arm1Positions = {
    initial: [0.439, 0.001, 0.365, 0.0, 1.57, 0.0],
    goal: [-0.182, 0.329, 0.217, 0.044, 1.57, 0.044]
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
      
      {/* 4行5列のグリッド */}
      <div className="button-grid">
        {Array.from({ length: 20 }).map((_, index) => (
          <GridButton 
            key={index + 1} 
            buttonNumber={index + 1}
          />
        ))}
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
        </div>
      </div>
    </div>
  );
}