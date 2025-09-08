import React, { useState, useEffect, useRef } from 'react';
import ROSLIB from 'roslib';
import './App.css';

// --- ROS 2 接続設定 ---
const ROSBRIDGE_SERVER_URL = "ws://192.168.10.102:9090";
const COMMAND_TOPIC_NAME = "/robot_command";
const COMMAND_MESSAGE_TYPE = "std_msgs/msg/String";
const POSE_TOPIC_NAME = "/button_command";
const POSE_MESSAGE_TYPE = "std_msgs/msg/String";
const TAP_PIXEL_TOPIC = "/tap/d415_pixel";
const TAP_PIXEL_MESSAGE_TYPE = "geometry_msgs/msg/Point";
const ARM1_UP_TOPIC = "/left_arm_up";
const ARM1_DOWN_TOPIC = "/left_arm_down";
const ARM2_UP_TOPIC = "/right_arm_up";
const ARM2_DOWN_TOPIC = "/right_arm_down";
const ARM1_GRAB_TOPIC = "/left_arm_close";
const ARM1_RELEASE_TOPIC = "/left_arm_open";
const ARM2_GRAB_TOPIC = "/right_arm_close";
const ARM2_RELEASE_TOPIC = "/right_arm_open";
const SEIRETU_TOPIC = "/seiretu";
const RED_SEIRETU_CONTROLLER_TOPIC = "/red_seiretu_controller/joint_trajectory";
const BLUE_SEIRETU_CONTROLLER_TOPIC = "/blue_seiretu_controller/joint_trajectory";
const UP_DOWN_MESSAGE_TYPE = "std_msgs/msg/String";
const SEIRETU_MESSAGE_TYPE = "std_msgs/msg/String";
const JOINT_TRAJECTORY_MESSAGE_TYPE = "trajectory_msgs/msg/JointTrajectory";

// リアルタイム制御用のトピック (アーム選択対応)
const REALTIME_CONTROL_MESSAGE_TYPE = "geometry_msgs/msg/Twist";

export default function App() {
  const [connectionStatus, setConnectionStatus] = useState("Disconnected");
  const [lastReceivedMessage, setLastReceivedMessage] = useState("");
  const [backgroundColor, setBackgroundColor] = useState("red");
  const [posePublisher, setPosePublisher] = useState(null);
  const [arm1UpPublisher, setArm1UpPublisher] = useState(null);
  const [arm1DownPublisher, setArm1DownPublisher] = useState(null);
  const [arm2UpPublisher, setArm2UpPublisher] = useState(null);
  const [arm2DownPublisher, setArm2DownPublisher] = useState(null);
  const [arm1GrabPublisher, setArm1GrabPublisher] = useState(null);
  const [arm1ReleasePublisher, setArm1ReleasePublisher] = useState(null);
  const [arm2GrabPublisher, setArm2GrabPublisher] = useState(null);
  const [arm2ReleasePublisher, setArm2ReleasePublisher] = useState(null);
  const [seiretuPublisher, setSeiretuPublisher] = useState(null);
  const [redSeiretuControllerPublisher, setRedSeiretuControllerPublisher] = useState(null);
  const [blueSeiretuControllerPublisher, setBlueSeiretuControllerPublisher] = useState(null);
  const [realtimeControlPublisher, setRealtimeControlPublisher] = useState(null);
  const [tapPixelPublisher, setTapPixelPublisher] = useState(null);
  const [cameraImageUrl, setCameraImageUrl] = useState<string>("http://192.168.10.102:8080/stream?topic=/camera/camera/color/image_raw");
  const [isCameraOpen, setIsCameraOpen] = useState<boolean>(false);
  const [clickedCoordinates, setClickedCoordinates] = useState<{x: number, y: number} | null>(null);
  const [selectedArm, setSelectedArm] = useState<"left" | "right" | "both">("left");
  const [isTimerRunning, setIsTimerRunning] = useState(false);
  const [timeLeft, setTimeLeft] = useState(180); // 3分 = 180秒
  
  // トピック監視用の状態
  const [publishedCommands, setPublishedCommands] = useState<Array<{topic: string, message: any, timestamp: string}>>([]);
  const [realtimeControlData, setRealtimeControlData] = useState<{linear: any, angular: any, timestamp: string} | null>(null);

  const ros = useRef(null);
  const publisher = useRef(null);
  const listener = useRef(null);
  const timerRef = useRef<NodeJS.Timeout | null>(null);
  const realtimeControlInterval = useRef<NodeJS.Timeout | null>(null);
  const currentRealtimeCommand = useRef<{x: number, y: number, z: number}>({x: 0, y: 0, z: 0});

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
      initializeUpDownPublishers();
      initializeGrabReleasePublishers();
      initializeSeiretuPublishers();
      initializeSeiretuControllerPublishers();
      initializeRealtimeControlPublisher();
      initializeTapPixelPublisher();
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
      // タイマーをクリア
      if (timerRef.current) {
        clearInterval(timerRef.current);
      }
      // リアルタイム制御のintervalもクリア
      if (realtimeControlInterval.current) {
        clearInterval(realtimeControlInterval.current);
      }
    };
  }, []);

  // タイマーの useEffect
  useEffect(() => {
    if (isTimerRunning && timeLeft > 0) {
      timerRef.current = setInterval(() => {
        setTimeLeft(prevTime => prevTime - 1);
      }, 1000);
    } else if (timeLeft === 0) {
      setIsTimerRunning(false);
    }

    return () => {
      if (timerRef.current) {
        clearInterval(timerRef.current);
      }
    };
  }, [isTimerRunning, timeLeft]);

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

  const initializeUpDownPublishers = () => {
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

  const initializeGrabReleasePublishers = () => {
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

  const initializeSeiretuPublishers = () => {
    const seiretuPub = new ROSLIB.Topic({
      ros: ros.current,
      name: SEIRETU_TOPIC,
      messageType: SEIRETU_MESSAGE_TYPE
    });
    setSeiretuPublisher(seiretuPub);
  };

  const initializeSeiretuControllerPublishers = () => {
    const redSeiretuControllerPub = new ROSLIB.Topic({
      ros: ros.current,
      name: RED_SEIRETU_CONTROLLER_TOPIC,
      messageType: JOINT_TRAJECTORY_MESSAGE_TYPE
    });
    setRedSeiretuControllerPublisher(redSeiretuControllerPub);

    const blueSeiretuControllerPub = new ROSLIB.Topic({
      ros: ros.current,
      name: BLUE_SEIRETU_CONTROLLER_TOPIC,
      messageType: JOINT_TRAJECTORY_MESSAGE_TYPE
    });
    setBlueSeiretuControllerPublisher(blueSeiretuControllerPub);
  };

  const initializeRealtimeControlPublisher = () => {
    // 左右アーム用のpublisherを初期化 (実際にはpublish時に動的に作成)
    // ここではstateの初期化のみ行う
    setRealtimeControlPublisher(null);
  };

  const initializeTapPixelPublisher = () => {
    const tapPixelPub = new ROSLIB.Topic({
      ros: ros.current,
      name: TAP_PIXEL_TOPIC,
      messageType: TAP_PIXEL_MESSAGE_TYPE
    });
    setTapPixelPublisher(tapPixelPub);
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

  const buttonPoseValues = {
    red: {
      1:"Red_Pose1",
      2:"Red_Pose2",
      3:"Red_Pose3",
      4:"Red_Pose4",
      5:"Red_Pose5",
      6:"Red_Pose6",
      7:"Red_Pose7",
      8:"Red_Pose8",
      9:"Red_Pose9",
      10:"Red_Pose10",
      11:"Red_Pose11",
      12:"Red_Pose12",
      13:"Red_Pose13",
      14:"Red_Pose14",
      15:"Red_Pose15",
      16:"Red_Pose16",
      17:"Red_Pose17",
      18:"Red_Pose18",
      19:"Red_Pose19",
      20:"Red_Pose20",
      21:"Red_Pose21",
      22:"Red_Pose22",
      23:"Red_Pose23",
      24:"Red_Pose24",
      25:"Red_Pose25",
      26:"Red_Pose26",
      27:"Red_Pose27",
      28:"Red_Pose28",
      29:"Red_Pose29",
      30:"Red_Pose30"
    },
    blue: {
      1:"Blue_Pose1",
      2:"Blue_Pose2",
      3:"Blue_Pose3",
      4:"Blue_Pose4",
      5:"Blue_Pose5",
      6:"Blue_Pose6",
      7:"Blue_Pose7",
      8:"Blue_Pose8",
      9:"Blue_Pose9",
      10:"Blue_Pose10",
      11:"Blue_Pose11",
      12:"Blue_Pose12",
      13:"Blue_Pose13",
      14:"Blue_Pose14",
      15:"Blue_Pose15",
      16:"Blue_Pose16",
      17:"Blue_Pose17",
      18:"Blue_Pose18",
      19:"Blue_Pose19",
      20:"Blue_Pose20",
      21:"Blue_Pose21",
      22:"Blue_Pose22",
      23:"Blue_Pose23",
      24:"Blue_Pose24",
      25:"Blue_Pose25",
      26:"Blue_Pose26",
      27:"Blue_Pose27",
      28:"Blue_Pose28",
      29:"Blue_Pose29",
      30:"Blue_Pose30"
    }
  };

  const armPositions = {
    red: {
      arm1: {
        initial: "Red_Left_Initial",
        goal: "Red_Left_Goal"
      },
      arm2: {
        initial: "Red_Right_Initial",
        goal: "Red_Right_Goal"
      }
    },
    blue: {
      arm1: {
        initial: "Blue_Left_Initial",
        goal: "Blue_Left_Goal"
      },
      arm2: {
        initial: "Blue_Right_Initial",
        goal: "Blue_Right_Goal"
      }
    }
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

  const handlePoseButtonClick = (buttonNumber) => {
    if (posePublisher && connectionStatus === 'Connected') {
      if (selectedArm === 'both') {
        // bothモード時は左右のペアで同時パブリッシュ
        let leftPoseNumber, rightPoseNumber;
        
        if (buttonNumber <= 5) {
          // Pose1-5: そのままとPose6-10
          leftPoseNumber = buttonNumber;
          rightPoseNumber = buttonNumber + 5;
        } else if (buttonNumber <= 10) {
          // Pose6-10: Pose11-15とPose16-20
          leftPoseNumber = buttonNumber + 5;
          rightPoseNumber = buttonNumber + 10;
        } else {
          // Pose11-15: Pose21-25とPose26-30
          leftPoseNumber = buttonNumber + 10;
          rightPoseNumber = buttonNumber + 15;
        }
        
        const leftPoseValue = buttonPoseValues[backgroundColor][leftPoseNumber] || `${backgroundColor}_Pose${leftPoseNumber}`;
        const rightPoseValue = buttonPoseValues[backgroundColor][rightPoseNumber] || `${backgroundColor}_Pose${rightPoseNumber}`;
        
        // 左アーム用にパブリッシュ
        const leftMessage = new ROSLIB.Message({
          data: leftPoseValue
        });
        posePublisher.publish(leftMessage);
        console.log(`🎯 Published LEFT pose to ${POSE_TOPIC_NAME}: "${leftPoseValue}"`);
        
        // 少し遅延を入れて右アーム用にパブリッシュ
        setTimeout(() => {
          const rightMessage = new ROSLIB.Message({
            data: rightPoseValue
          });
          posePublisher.publish(rightMessage);
          console.log(`🎯 Published RIGHT pose to ${POSE_TOPIC_NAME}: "${rightPoseValue}"`);
        }, 10);
        
        // パブリッシュログに追加（両方記録）
        setPublishedCommands(prev => [
          {
            topic: POSE_TOPIC_NAME,
            message: `${leftPoseValue} + ${rightPoseValue}`,
            timestamp: new Date().toLocaleTimeString()
          },
          ...prev.slice(0, 4) // 最新5件のみ保持
        ]);
      } else {
        // 単一アームモード (left or right)
        let poseToPublish = buttonNumber; // デフォルトは元の番号

        if (selectedArm === 'right') {
          // 右アーム選択時のマッピング
          if (buttonNumber >= 1 && buttonNumber <= 5) {
            poseToPublish = buttonNumber + 5; // Pose1-5 -> Pose6-10
          } else if (buttonNumber >= 6 && buttonNumber <= 10) {
            poseToPublish = buttonNumber + 10; // Pose6-10 -> Pose16-20
          } else if (buttonNumber >= 11 && buttonNumber <= 15) {
            poseToPublish = buttonNumber + 15; // Pose11-15 -> Pose26-30
          }
        } else if (selectedArm === 'left') {
          // 左アーム選択時のマッピング
          if (buttonNumber >= 6 && buttonNumber <= 10) {
            poseToPublish = buttonNumber + 5; // Pose6-10 -> Pose11-15
          } else if (buttonNumber >= 11 && buttonNumber <= 15) {
            poseToPublish = buttonNumber + 10; // Pose11-15 -> Pose21-25
          }
        }
        
        const poseValue = buttonPoseValues[backgroundColor][poseToPublish] || `${backgroundColor}_Pose${poseToPublish}`;
        const message = new ROSLIB.Message({
          data: poseValue
        });
        posePublisher.publish(message);
        console.log(`🎯 Published pose to ${POSE_TOPIC_NAME}: "${poseValue}"`);
        
        // パブリッシュログに追加
        setPublishedCommands(prev => [
          {
            topic: POSE_TOPIC_NAME,
            message: poseValue,
            timestamp: new Date().toLocaleTimeString()
          },
          ...prev.slice(0, 4) // 最新5件のみ保持
        ]);
      }
    } else {
      console.warn(`Cannot send pose. ROS Status: ${connectionStatus}`);
    }
  };

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

  const handleArm1Grab = () => {
    if (arm1GrabPublisher && connectionStatus === 'Connected') {
      const message = new ROSLIB.Message({
        data: "close"
      });
      arm1GrabPublisher.publish(message);
      console.log(`✊ Published to ${ARM1_GRAB_TOPIC}: "close"`);
    } else {
      console.warn(`Cannot send arm1 grab command. ROS Status: ${connectionStatus}`);
    }
  };

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
  };

  const handleArm2Grab = () => {
    if (arm2GrabPublisher && connectionStatus === 'Connected') {
      const message = new ROSLIB.Message({
        data: "close"
      });
      arm2GrabPublisher.publish(message);
      console.log(`✊ Published to ${ARM2_GRAB_TOPIC}: "close"`);
    } else {
      console.warn(`Cannot send arm2 grab command. ROS Status: ${connectionStatus}`);
    }
  };

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
  };

  const publishJointTrajectory = (position, color) => {
    let sliderValue;
    
    // ポジションに応じてslider値を設定
    switch(position) {
      case 'left':
        sliderValue = -0.071;
        break;
      case 'middle':
        sliderValue = -0.264;
        break;
      case 'right':
        sliderValue = -0.457;
        break;
      default:
        sliderValue = 0.0;
    }
    
    const publisher = color === 'red' ? redSeiretuControllerPublisher : blueSeiretuControllerPublisher;
    const jointName = color === 'red' ? 'red_slider' : 'blue_slider';
    const topicName = color === 'red' ? RED_SEIRETU_CONTROLLER_TOPIC : BLUE_SEIRETU_CONTROLLER_TOPIC;
    
    if (publisher && connectionStatus === 'Connected') {
      const message = new ROSLIB.Message({
        joint_names: [jointName],
        points: [{
          positions: [sliderValue],
          time_from_start: {
            sec: 1,
            nanosec: 0
          }
        }]
      });
      publisher.publish(message);
      console.log(`🎛️ Published JointTrajectory to ${topicName}: ${jointName}=${sliderValue} (${position})`);
    }
  };

  const handleSeiretuLeft = () => {
    if (seiretuPublisher && connectionStatus === 'Connected') {
      const commandValue = `${backgroundColor}_seiretu_left`;
      const message = new ROSLIB.Message({
        data: commandValue
      });
      seiretuPublisher.publish(message);
      console.log(`📏 Published to ${SEIRETU_TOPIC}: "${commandValue}"`);
      
      // JointTrajectoryでslider jointの値も更新
      publishJointTrajectory('left', backgroundColor);
    } else {
      console.warn(`Cannot send seiretu left command. ROS Status: ${connectionStatus}`);
    }
  };

  const handleSeiretuMiddle = () => {
    if (seiretuPublisher && connectionStatus === 'Connected') {
      const commandValue = `${backgroundColor}_seiretu_middle`;
      const message = new ROSLIB.Message({
        data: commandValue
      });
      seiretuPublisher.publish(message);
      console.log(`📏 Published to ${SEIRETU_TOPIC}: "${commandValue}"`);
      
      // JointTrajectoryでslider jointの値も更新
      publishJointTrajectory('middle', backgroundColor);
    } else {
      console.warn(`Cannot send seiretu middle command. ROS Status: ${connectionStatus}`);
    }
  };

  const handleSeiretuRight = () => {
    if (seiretuPublisher && connectionStatus === 'Connected') {
      const commandValue = `${backgroundColor}_seiretu_right`;
      const message = new ROSLIB.Message({
        data: commandValue
      });
      seiretuPublisher.publish(message);
      console.log(`📏 Published to ${SEIRETU_TOPIC}: "${commandValue}"`);
      
      // JointTrajectoryでslider jointの値も更新
      publishJointTrajectory('right', backgroundColor);
    } else {
      console.warn(`Cannot send seiretu right command. ROS Status: ${connectionStatus}`);
    }
  };


  // 変更：アーム1ゴール位置
  const handleArm1Goal = () => {
    if (posePublisher && connectionStatus === 'Connected') {
      const poseValue = armPositions[backgroundColor].arm1.goal;
      const message = new ROSLIB.Message({
        data: poseValue
      });
      posePublisher.publish(message);
      console.log(`🎯 Published Arm1 Goal pose to ${POSE_TOPIC_NAME}: "${poseValue}"`);
    } else {
      console.warn(`Cannot send pose. ROS Status: ${connectionStatus}`);
    }
  };


  // 変更：アーム2ゴール位置
  const handleArm2Goal = () => {
    if (posePublisher && connectionStatus === 'Connected') {
      const poseValue = armPositions[backgroundColor].arm2.goal;
      const message = new ROSLIB.Message({
        data: poseValue
      });
      posePublisher.publish(message);
      console.log(`🎯 Published Arm2 Goal pose to ${POSE_TOPIC_NAME}: "${poseValue}"`);
    } else {
      console.warn(`Cannot send pose. ROS Status: ${connectionStatus}`);
    }
  };

  const toggleBackgroundColor = () => {
    setBackgroundColor(prevColor => prevColor === "red" ? "blue" : "red");
  };

  const toggleArm = () => {
    setSelectedArm(prevArm => {
      if (prevArm === "left") return "right";
      if (prevArm === "right") return "both";
      return "left";
    });
  };

  const toggleCamera = () => {
    setIsCameraOpen(prev => !prev);
    if (!isCameraOpen) {
      setClickedCoordinates(null); // カメラを開く時に座標をリセット
    }
  };

  const toggleTimer = () => {
    if (isTimerRunning) {
      // Stopボタンを押したときに3分に戻す
      setIsTimerRunning(false);
      setTimeLeft(180); // 3分 = 180秒
      if (timerRef.current) {
        clearInterval(timerRef.current);
      }
    } else {
      setIsTimerRunning(true);
    }
  };

  const resetTimer = () => {
    setIsTimerRunning(false);
    setTimeLeft(180); // 3分 = 180秒
    if (timerRef.current) {
      clearInterval(timerRef.current);
    }
  };

  const formatTime = (seconds: number) => {
    const mins = Math.floor(seconds / 60);
    const secs = seconds % 60;
    return `${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
  };

  const getVisiblePoses = () => {
    // both選択時は1-15を表示
    return [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];
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

  // 新しいリアルタイム制御開始関数
  const startRealtimeControl = (linear_x: number, linear_y: number, angular_z: number) => {
    if (ros.current && connectionStatus === 'Connected') {
      // 現在のコマンドを更新
      currentRealtimeCommand.current = { x: linear_x, y: linear_y, z: angular_z };
      
      // 既存のインターバルがあればクリア
      if (realtimeControlInterval.current) {
        clearInterval(realtimeControlInterval.current);
      }
      
      // 即座に最初のメッセージを送信
      publishRealtimeControlMessage(linear_x, linear_y, angular_z);
      
      // 継続的にメッセージを送信（100ms間隔）
      realtimeControlInterval.current = setInterval(() => {
        publishRealtimeControlMessage(
          currentRealtimeCommand.current.x,
          currentRealtimeCommand.current.y,
          currentRealtimeCommand.current.z
        );
      }, 10);
    }
  };

  // リアルタイム制御停止関数
  const stopRealtimeControl = () => {
    // インターバルをクリア
    if (realtimeControlInterval.current) {
      clearInterval(realtimeControlInterval.current);
      realtimeControlInterval.current = null;
    }
    
    // 停止メッセージを送信
    publishRealtimeControlMessage(0, 0, 0);
    
    // 現在のコマンドをリセット
    currentRealtimeCommand.current = { x: 0, y: 0, z: 0 };
  };

  // 実際のメッセージ送信関数
  const publishRealtimeControlMessage = (linear_x: number, linear_y: number, angular_z: number) => {
    if (ros.current && connectionStatus === 'Connected') {
      // アーム選択に応じてトピックを変更
      const topicName = selectedArm === "left" ? "/left_arm_realtime_control" : 
                       selectedArm === "right" ? "/right_arm_realtime_control" : 
                       "/both_arms_realtime_control";
      
      // 新しいpublisherを作成してパブリッシュ
      const publisher = new ROSLIB.Topic({
        ros: ros.current,
        name: topicName,
        messageType: "geometry_msgs/msg/Twist"
      });
      
      const message = new ROSLIB.Message({
        linear: {
          x: linear_x,
          y: linear_y,
          z: 0.0
        },
        angular: {
          x: 0.0,
          y: 0.0,
          z: angular_z
        }
      });
      publisher.publish(message);
      
      // ログは最初の送信時のみ出力
      if (linear_x !== 0 || linear_y !== 0 || angular_z !== 0) {
        console.log(`🎮 Real-time control (${selectedArm} arm): linear(${linear_x}, ${linear_y}), angular(${angular_z})`);
        
        // リアルタイム制御データを更新
        setRealtimeControlData({
          linear: {x: linear_x, y: linear_y, z: 0.0},
          angular: {x: 0.0, y: 0.0, z: angular_z},
          timestamp: new Date().toLocaleTimeString()
        });
      }
    }
  };

  // D-padコントローラーコンポーネント
  const DPadController = () => {
    // フィールドに応じて方向を調整
    const getLinearValues = (baseX: number, baseY: number) => {
      const multiplier = backgroundColor === "red" ? 1 : -1;
      return {
        x: baseX * multiplier,
        y: baseY * multiplier
      };
    };

    return (
      <div className="dpad-container">
        <div className="dpad">
          {/* 上 */}
          <button 
            className="dpad-button dpad-up"
            onMouseDown={() => {
              const values = getLinearValues(-1.0, 0);
              startRealtimeControl(values.x, values.y, 0);
            }}
            onMouseUp={() => stopRealtimeControl()}
            onMouseLeave={() => stopRealtimeControl()}
            onTouchStart={(e) => {
              e.preventDefault();
              const values = getLinearValues(-1.0, 0);
              startRealtimeControl(values.x, values.y, 0);
            }}
            onTouchEnd={(e) => {
              e.preventDefault();
              stopRealtimeControl();
            }}
            disabled={connectionStatus !== 'Connected'}
          >
            ⬆
          </button>
          {/* 左 */}
          <button 
            className="dpad-button dpad-left"
            onMouseDown={() => {
              const values = getLinearValues(0, -1.0);
              startRealtimeControl(values.x, values.y, 0);
            }}
            onMouseUp={() => stopRealtimeControl()}
            onMouseLeave={() => stopRealtimeControl()}
            onTouchStart={(e) => {
              e.preventDefault();
              const values = getLinearValues(0, -1.0);
              startRealtimeControl(values.x, values.y, 0);
            }}
            onTouchEnd={(e) => {
              e.preventDefault();
              stopRealtimeControl();
            }}
            disabled={connectionStatus !== 'Connected'}
          >
            ⬅
          </button>
          {/* 中央 */}
          <div className="dpad-center"></div>
          {/* 右 */}
          <button 
            className="dpad-button dpad-right"
            onMouseDown={() => {
              const values = getLinearValues(0, 1.0);
              startRealtimeControl(values.x, values.y, 0);
            }}
            onMouseUp={() => stopRealtimeControl()}
            onMouseLeave={() => stopRealtimeControl()}
            onTouchStart={(e) => {
              e.preventDefault();
              const values = getLinearValues(0, 1.0);
              startRealtimeControl(values.x, values.y, 0);
            }}
            onTouchEnd={(e) => {
              e.preventDefault();
              stopRealtimeControl();
            }}
            disabled={connectionStatus !== 'Connected'}
          >
            ➡
          </button>
          {/* 下 */}
          <div className="dpad-down-container">
            <button 
              className="dpad-button dpad-down"
              onMouseDown={() => {
                const values = getLinearValues(1.0, 0);
                startRealtimeControl(values.x, values.y, 0);
              }}
              onMouseUp={() => stopRealtimeControl()}
              onMouseLeave={() => stopRealtimeControl()}
              onTouchStart={(e) => {
                e.preventDefault();
                const values = getLinearValues(1.0, 0);
                startRealtimeControl(values.x, values.y, 0);
              }}
              onTouchEnd={(e) => {
                e.preventDefault();
                stopRealtimeControl();
              }}
              disabled={connectionStatus !== 'Connected'}
            >
              ⬇
            </button>
            <div className="dpad-label">XY Movement</div>
          </div>
        </div>
      </div>
    );
  };

  // ヨー角制御用曲線矢印コンポーネント
  const YawController = () => (
    <div className="yaw-controller">
      <div className="yaw-buttons-container">
        <button 
          className="yaw-button yaw-left"
          onMouseDown={() => startRealtimeControl(0, 0, 2.0)}
          onMouseUp={() => stopRealtimeControl()}
          onMouseLeave={() => stopRealtimeControl()}
          onTouchStart={(e) => {
            e.preventDefault();
            startRealtimeControl(0, 0, 2.0);
          }}
          onTouchEnd={(e) => {
            e.preventDefault();
            stopRealtimeControl();
          }}
          disabled={connectionStatus !== 'Connected'}
        >
          <svg width="40" height="40" viewBox="0 0 40 40">
            <path 
              d="M 30 15 A 8 8 0 1 0 30 25" 
              stroke="currentColor" 
              strokeWidth="3" 
              fill="none"
              markerEnd="url(#arrowhead-ccw)"
            />
            <defs>
              <marker id="arrowhead-ccw" markerWidth="10" markerHeight="7" 
                refX="9" refY="3.5" orient="auto ">
                <polygon points="0 0, 10 3.5, 0 7" fill="currentColor" />
              </marker>
            </defs>
          </svg>
        </button>
        <button 
          className="yaw-button yaw-right"
          onMouseDown={() => startRealtimeControl(0, 0, -2.0)}
          onMouseUp={() => stopRealtimeControl()}
          onMouseLeave={() => stopRealtimeControl()}
          onTouchStart={(e) => {
            e.preventDefault();
            startRealtimeControl(0, 0, -2.0);
          }}
          onTouchEnd={(e) => {
            e.preventDefault();
            stopRealtimeControl();
          }}
          disabled={connectionStatus !== 'Connected'}
        >
          <svg width="40" height="40" viewBox="0 0 40 40">
            <path 
              d="M 10 15 A 8 8 0 1 1 10 25" 
              stroke="currentColor" 
              strokeWidth="3" 
              fill="none"
              markerEnd="url(#arrowhead-cw)"
            />
            <defs>
              <marker id="arrowhead-cw" markerWidth="10" markerHeight="7" 
                refX="9" refY="3.5" orient="auto">
                <polygon points="0 0, 10 3.5, 0 7" fill="currentColor" />
              </marker>
            </defs>
          </svg>
        </button>
      </div>
      <div className="yaw-label">Yaw Rotation</div>
    </div>
  );

  const CameraView = () => (
    <div className="camera-view-container-fullscreen">
      <div className="camera-image-wrapper">
        <img 
          src="http://192.168.10.102:8080/stream?topic=/camera/camera/color/image_raw" 
          alt="Camera Feed" 
          className={`camera-image ${selectedArm === "left" ? "camera-image-left" : "camera-image-right"}`}
          onClick={handleImageClick}
          onError={(e) => {
            console.log('カメラ映像読み込みエラー');
            const img = e.target as HTMLImageElement;
            setTimeout(() => {
              img.src = "http://192.168.10.102:8080/stream?topic=/camera/camera/color/image_raw";
            }, 3000);
          }}
        />
        {clickedCoordinates && (
          <div className="coordinate-display-inline">
            📍 ({clickedCoordinates.x}, {clickedCoordinates.y})
          </div>
        )}
      </div>
      <div className="side-controls-expanded">
        <div className="controller-section-horizontal">
          <div className="left-controller-section">
            <YawController />
            <DPadController />
          </div>
          <div className="right-controller-section">
            {selectedArm !== "right" && (
              <div className="up-down-buttons vertical">
                <button 
                  className="up-down-button up-button"
                  onClick={handleArm1UpButtonClick}
                  disabled={connectionStatus !== 'Connected'}
                >
                  ⬆️ LEFT UP
                </button>
                <button 
                  className="up-down-button down-button"
                  onClick={handleArm1DownButtonClick}
                  disabled={connectionStatus !== 'Connected'}
                >
                  ⬇️ LEFT DOWN
                </button>
              </div>
            )}
            
            {selectedArm !== "left" && (
              <div className="up-down-buttons vertical">
                <button 
                  className="up-down-button up-button"
                  onClick={handleArm2UpButtonClick}
                  disabled={connectionStatus !== 'Connected'}
                >
                  ⬆️ RIGHT UP
                </button>
                <button 
                  className="up-down-button down-button"
                  onClick={handleArm2DownButtonClick}
                  disabled={connectionStatus !== 'Connected'}
                >
                  ⬇️ RIGHT DOWN
                </button>
              </div>
            )}
          </div>
        </div>
      </div>
    </div>
  );

  const handleImageClick = (event: React.MouseEvent<HTMLImageElement>) => {
    const img = event.currentTarget;
    const rect = img.getBoundingClientRect();
    
    // 画像の実際の表示サイズを取得（余白を除く）
    const imageAspectRatio = img.naturalWidth / img.naturalHeight;
    const containerAspectRatio = rect.width / rect.height;
    
    let actualImageWidth, actualImageHeight, offsetX, offsetY;
    
    if (containerAspectRatio > imageAspectRatio) {
      // コンテナが画像より横長の場合（左右に余白）
      actualImageHeight = rect.height;
      actualImageWidth = rect.height * imageAspectRatio;
      offsetX = (rect.width - actualImageWidth) / 2;
      offsetY = 0;
    } else {
      // コンテナが画像より縦長の場合（上下に余白）
      actualImageWidth = rect.width;
      actualImageHeight = rect.width / imageAspectRatio;
      offsetX = 0;
      offsetY = (rect.height - actualImageHeight) / 2;
    }
    
    // クリック位置から余白を除いた座標を計算
    const relativeX = event.clientX - rect.left - offsetX;
    const relativeY = event.clientY - rect.top - offsetY;
    
    // 余白内をクリックした場合は無視
    if (relativeX < 0 || relativeY < 0 || relativeX > actualImageWidth || relativeY > actualImageHeight) {
      return;
    }
    
    // 実際の画像ピクセル座標に変換
    const x = Math.floor(relativeX * (img.naturalWidth / actualImageWidth));
    const y = Math.floor(relativeY * (img.naturalHeight / actualImageHeight));
    
    setClickedCoordinates({ x, y });
    console.log(`カメラ画像クリック座標: (${x}, ${y})`);
    
    // ROSトピックにピクセル座標を送信
    if (tapPixelPublisher && connectionStatus === 'Connected') {
      const message = new ROSLIB.Message({
        x: x,
        y: y,
        z: 0
      });
      tapPixelPublisher.publish(message);
      console.log(`📍 Published pixel coordinates to ${TAP_PIXEL_TOPIC}: (${x}, ${y})`);
    }
  };

  const CameraLargeView = () => (
    <div style={{ 
      position: 'fixed', 
      top: '0', 
      left: '0', 
      width: '100vw',
      height: '100vh',
      zIndex: 2000,
      background: 'rgba(0, 0, 0, 0.9)',
      display: 'flex',
      justifyContent: 'center',
      alignItems: 'center'
    }}>
      <button 
        style={{ 
          position: 'absolute',
          top: '20px',
          right: '20px',
          background: 'rgba(231, 76, 60, 0.9)',
          color: 'white',
          border: 'none',
          padding: '12px 20px',
          borderRadius: '25px',
          fontSize: '1rem',
          fontWeight: '700',
          cursor: 'pointer',
          boxShadow: '0 4px 15px rgba(0,0,0,0.3)',
          transition: 'all 0.3s ease',
          zIndex: 2001
        }}
        onClick={() => setIsCameraOpen(false)}
        onMouseOver={(e) => {
          e.target.style.background = 'rgba(192, 57, 43, 0.9)';
          e.target.style.transform = 'scale(1.05)';
        }}
        onMouseOut={(e) => {
          e.target.style.background = 'rgba(231, 76, 60, 0.9)';
          e.target.style.transform = 'scale(1)';
        }}
      >
        ✕ Close Camera
      </button>
      
      {/* 座標表示エリア */}
      {clickedCoordinates && (
        <div className="coordinate-display">
          <p>📍 Clicked Coordinates: ({clickedCoordinates.x}, {clickedCoordinates.y})</p>
        </div>
      )}
      
      <div style={{
        background: 'rgba(255, 255, 255, 0.95)',
        padding: '30px',
        borderRadius: '20px',
        boxShadow: '0 20px 60px rgba(0,0,0,0.4)',
        backdropFilter: 'blur(10px)'
      }}>
        <h2 style={{ 
          margin: '0 0 20px 0', 
          color: '#2980b9',
          fontSize: '1.8rem',
          fontWeight: '700',
          textAlign: 'center'
        }}>
          📷 カメラ映像 (大画面) - クリックで座標表示
        </h2>
        <img 
          src="http://192.168.10.102:8080/stream?topic=/camera/camera/color/image_raw" 
          alt="Camera Feed Large View" 
          className={selectedArm === "left" ? "camera-image-left-large" : "camera-image-right-large"}
          style={{ 
            width: '80vw', 
            height: '60vh', 
            maxWidth: '1200px',
            maxHeight: '800px',
            objectFit: 'contain',
            borderRadius: '15px',
            border: '2px solid #3498db',
            boxShadow: '0 10px 30px rgba(0,0,0,0.2)',
            cursor: 'crosshair'
          }}
          onClick={handleImageClick}
          onError={(e) => {
            console.log('カメライン映像読み込みエラー (大画面)');
            const img = e.target as HTMLImageElement;
            setTimeout(() => {
              img.src = "http://192.168.10.102:8080/stream?topic=/camera/camera/color/image_raw";
            }, 3000);
          }}
        />
        <p style={{ 
          fontSize: '1rem', 
          color: '#6c757d', 
          margin: '15px 0 0 0',
          textAlign: 'center',
          fontWeight: '500'
        }}>
          Realsense Camera Feed - Large View (画像をクリックしてピクセル座標を取得)
        </p>
      </div>
    </div>
  );
  return (
    <div 
      className="app-container"
      data-background={backgroundColor}
    >
      <header className="app-header">
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
            className="toggle-button arm-toggle"
            onClick={toggleArm}
          >
            🦾 Arm: {selectedArm === "left" ? "左" : selectedArm === "right" ? "右" : "両方"}
          </button>
          <button 
            className="toggle-button camera-toggle"
            onClick={toggleCamera}
          >
            📷 Camera: {isCameraOpen ? "ON" : "OFF"}
          </button>
          <button 
            className="toggle-button"
            onClick={toggleBackgroundColor}
          >
            🎨 Field: {backgroundColor === "red" ? "赤" : "青"}
          </button>
          <button 
            className={`toggle-button timer-toggle ${isTimerRunning ? 'timer-running' : ''}`}
            onClick={toggleTimer}
          >
            ⏱️ {isTimerRunning ? "Stop" : "Start"}
          </button>
          <div className="timer-display">
            {formatTime(timeLeft)}
          </div>
        </div>
      </header>
      
      <div className="pose-grid-container">
        {isCameraOpen ? (
          <CameraView />
        ) : (
          <div className="camera-view-container">
            <div className="camera-image-wrapper" style={{ height: '100%', width: '100%', display: 'flex', flexDirection: 'column', padding: '0' }}>
              <div style={{ display: 'flex', flexDirection: 'column', gap: '1rem', height: '100%', width: '100%' }}>
                <div className="pose-grid" style={{ 
                  gap: 'clamp(0.3rem, 0.5vw, 0.6rem)',
                  flex: '1',
                  width: '100%'
                }}>
                  {getVisiblePoses().map((buttonNumber) => (
                    <GridButton 
                      key={buttonNumber} 
                      buttonNumber={buttonNumber}
                    />
                  ))}
                </div>
                
                {/* トピック監視GUI */}
                <div className="topic-monitor-section" style={{
                  background: 'rgba(255, 255, 255, 0.95)',
                  borderRadius: '15px',
                  padding: '0.8rem',
                  boxShadow: '0 4px 15px rgba(0, 0, 0, 0.1)',
                  backdropFilter: 'blur(10px)',
                  border: '1px solid rgba(255, 255, 255, 0.3)',
                  height: '120px',
                  overflowY: 'auto',
                  flex: '0 0 120px',
                  width: '100%'
                }}>
                  <h3 style={{
                    color: '#2c3e50', 
                    fontSize: '1rem', 
                    margin: '0 0 0.6rem 0',
                    textAlign: 'center',
                    fontWeight: '700'
                  }}>
                    📡 Topic Monitor
                  </h3>
                  
                  <div style={{ display: 'flex', gap: '0.8rem', height: 'calc(100% - 1.6rem)' }}>
                    {/* リアルタイム制御表示 */}
                    <div style={{ flex: '1' }}>
                      <h4 style={{ color: '#3498db', fontSize: '0.85rem', marginBottom: '0.3rem' }}>🎮 Realtime Control</h4>
                      {realtimeControlData ? (
                        <div style={{
                          background: 'rgba(52, 152, 219, 0.1)',
                          padding: '0.5rem',
                          borderRadius: '6px',
                          border: '1px solid rgba(52, 152, 219, 0.3)',
                          fontSize: '0.7rem'
                        }}>
                          <div><strong>Linear:</strong> x:{realtimeControlData.linear.x.toFixed(1)} y:{realtimeControlData.linear.y.toFixed(1)}</div>
                          <div><strong>Angular:</strong> z:{realtimeControlData.angular.z.toFixed(1)}</div>
                          <div style={{ color: '#7f8c8d', fontSize: '0.65rem', marginTop: '0.3rem' }}>
                            {realtimeControlData.timestamp}
                          </div>
                        </div>
                      ) : (
                        <div style={{
                          background: 'rgba(149, 165, 166, 0.1)',
                          padding: '0.5rem',
                          borderRadius: '6px',
                          textAlign: 'center',
                          color: '#95a5a6',
                          fontStyle: 'italic',
                          fontSize: '0.7rem'
                        }}>
                          No realtime data
                        </div>
                      )}
                    </div>
                    
                    {/* パブリッシュされたコマンド表示 */}
                    <div style={{ flex: '1' }}>
                      <h4 style={{ color: '#27ae60', fontSize: '0.85rem', marginBottom: '0.3rem' }}>📤 Published Commands</h4>
                      <div style={{ display: 'flex', flexDirection: 'column', gap: '0.3rem', height: 'calc(100% - 1.2rem)', overflowY: 'auto' }}>
                        {publishedCommands.length > 0 ? publishedCommands.slice(0, 2).map((cmd, index) => (
                          <div key={index} style={{
                            background: 'rgba(46, 204, 113, 0.1)',
                            padding: '0.4rem',
                            borderRadius: '4px',
                            border: '1px solid rgba(46, 204, 113, 0.3)',
                            fontSize: '0.65rem'
                          }}>
                            <div style={{ fontWeight: 'bold', color: '#27ae60', marginBottom: '0.2rem' }}>
                              {cmd.topic.split('/').pop()}: {cmd.message}
                            </div>
                            <div style={{ color: '#7f8c8d', fontSize: '0.6rem' }}>
                              {cmd.timestamp}
                            </div>
                          </div>
                        )) : (
                          <div style={{
                            background: 'rgba(149, 165, 166, 0.1)',
                            padding: '0.5rem',
                            borderRadius: '6px',
                            textAlign: 'center',
                            color: '#95a5a6',
                            fontStyle: 'italic',
                            fontSize: '0.7rem'
                          }}>
                            No commands published
                          </div>
                        )}
                      </div>
                    </div>
                  </div>
                </div>
              </div>
            </div>

            <div className="side-controls-expanded">
              <div className="controller-section-horizontal">
                <div className="left-controller-section">
                  <YawController />
                  <DPadController />
                </div>
                <div className="right-controller-section">
                  {selectedArm !== "right" && (
                    <div className="up-down-buttons vertical">
                      <button 
                        className="up-down-button up-button"
                        onClick={handleArm1UpButtonClick}
                        disabled={connectionStatus !== 'Connected'}
                      >
                        ⬆️ LEFT UP
                      </button>
                      <button 
                        className="up-down-button down-button"
                        onClick={handleArm1DownButtonClick}
                        disabled={connectionStatus !== 'Connected'}
                      >
                        ⬇️ LEFT DOWN
                      </button>
                    </div>
                  )}
                  
                  {selectedArm !== "left" && (
                    <div className="up-down-buttons vertical">
                      <button 
                        className="up-down-button up-button"
                        onClick={handleArm2UpButtonClick}
                        disabled={connectionStatus !== 'Connected'}
                      >
                        ⬆️ RIGHT UP
                      </button>
                      <button 
                        className="up-down-button down-button"
                        onClick={handleArm2DownButtonClick}
                        disabled={connectionStatus !== 'Connected'}
                      >
                        ⬇️ RIGHT DOWN
                      </button>
                    </div>
                  )}
                </div>
              </div>
            </div>
          </div>
        )}
      </div>

      <div className="bottom-control-area">
        {selectedArm !== "right" && (
          <div className="arm-controls single-arm">
            <button 
              className="arm-button goal-button"
              onClick={handleArm1Goal}
              disabled={connectionStatus !== 'Connected'}
            >
              🎯 ゴール
            </button>
            
            <button 
              className="arm-button grab-button"
              onClick={handleArm1Grab}
              disabled={connectionStatus !== 'Connected'}
            >
              ✊ 掴む
            </button>
            
            <button 
              className="arm-button release-button"
              onClick={handleArm1Release}
              disabled={connectionStatus !== 'Connected'}
            >
              🖐️ 離す
            </button>
            
            <button 
              className="arm-button seiretu-button seiretu-left"
              onClick={handleSeiretuLeft}
              disabled={connectionStatus !== 'Connected'}
            >
              📏 左
            </button>
            
            <button 
              className="arm-button seiretu-button seiretu-middle"
              onClick={handleSeiretuMiddle}
              disabled={connectionStatus !== 'Connected'}
            >
              📏 中央
            </button>
            
            <button 
              className="arm-button seiretu-button seiretu-right"
              onClick={handleSeiretuRight}
              disabled={connectionStatus !== 'Connected'}
            >
              📏 右
            </button>
          </div>
        )}
        
        {selectedArm !== "left" && (
          <div className="arm-controls single-arm">
            <button 
              className="arm-button goal-button"
              onClick={handleArm2Goal}
              disabled={connectionStatus !== 'Connected'}
            >
              🎯 ゴール
            </button>
            
            <button 
              className="arm-button grab-button"
              onClick={handleArm2Grab}
              disabled={connectionStatus !== 'Connected'}
            >
              ✊ 掴む
            </button>
            
            <button 
              className="arm-button release-button"
              onClick={handleArm2Release}
              disabled={connectionStatus !== 'Connected'}
            >
              🖐️ 離す
            </button>
            
            <button 
              className="arm-button seiretu-button seiretu-left"
              onClick={handleSeiretuLeft}
              disabled={connectionStatus !== 'Connected'}
            >
              📏 左
            </button>
            
            <button 
              className="arm-button seiretu-button seiretu-middle"
              onClick={handleSeiretuMiddle}
              disabled={connectionStatus !== 'Connected'}
            >
              📏 中央
            </button>
            
            
            <button 
              className="arm-button seiretu-button seiretu-right"
              onClick={handleSeiretuRight}
              disabled={connectionStatus !== 'Connected'}
            >
              📏 右
            </button>
          </div>
        )}
      </div>
    </div>
  );
}