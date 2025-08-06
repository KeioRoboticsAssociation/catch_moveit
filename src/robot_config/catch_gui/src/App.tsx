import React, { useState, useEffect, useRef } from 'react';
import ROSLIB from 'roslib';
import './App.css';

// --- ROS 2 接続設定 ---
const ROSBRIDGE_SERVER_URL = "ws://192.168.1.5:9090";
const COMMAND_TOPIC_NAME = "/robot_command";
const COMMAND_MESSAGE_TYPE = "std_msgs/msg/String";

export default function App() {
  const [connectionStatus, setConnectionStatus] = useState("Disconnected");
  const [lastReceivedMessage, setLastReceivedMessage] = useState("");
  const [backgroundColor, setBackgroundColor] = useState("red");

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

  const handleButtonClick = (commandText) => {
    if (publisher.current && connectionStatus === 'Connected') {
      const message = new ROSLIB.Message({
        data: commandText
      });
      publisher.current.publish(message);
      // ここに追加：ターミナルログにpublish内容を表示
      console.log(`🚀 Published to ${COMMAND_TOPIC_NAME}: "${commandText}"`);
    } else {
      console.warn(`Cannot send command. ROS Status: ${connectionStatus}`);
    }
  };

  const toggleBackgroundColor = () => {
    setBackgroundColor(prevColor => prevColor === "red" ? "blue" : "red");
  };

  const GridButton = ({ text, onClick }) => (
    <button 
      className="grid-button"
      onClick={onClick}
      disabled={connectionStatus !== 'Connected'}
    >
      {text}
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
            key={index} 
            text={`Button ${index + 1}`} 
            onClick={() => handleButtonClick(`Button ${index + 1}`)}
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
            onClick={() => handleButtonClick("アーム1 初期位置")}
            disabled={connectionStatus !== 'Connected'}
          >
            アーム1<br/>初期位置
          </button>
          
          {/* アーム1のゴールボタン */}
          <button 
            className="arm-button"
            onClick={() => handleButtonClick("アーム1 ゴール")}
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
            onClick={() => handleButtonClick("アーム2 初期位置")}
            disabled={connectionStatus !== 'Connected'}
          >
            アーム2<br/>初期位置
          </button>
          
          {/* アーム2のゴールボタン */}
          <button 
            className="arm-button"
            onClick={() => handleButtonClick("アーム2 ゴール")}
            disabled={connectionStatus !== 'Connected'}
          >
            アーム2<br/>ゴール
          </button>
        </div>
      </div>
    </div>
  );
}