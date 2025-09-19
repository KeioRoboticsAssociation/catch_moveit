import React, { useState, useEffect, useRef } from 'react';
import ROSLIB from 'roslib';
import './App.css';

// --- ROS 2 接続設定 ---
const ROSBRIDGE_SERVER_URL = "ws://192.168.100.3:9090";
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
  const [leftRealtimeControlPublisher, setLeftRealtimeControlPublisher] = useState<ROSLIB.Topic | null>(null);
  const [rightRealtimeControlPublisher, setRightRealtimeControlPublisher] = useState<ROSLIB.Topic | null>(null);
  const [tapPixelPublisher, setTapPixelPublisher] = useState(null);
  const [cameraImageUrl, setCameraImageUrl] = useState<string>("http://192.168.100.3:8080/stream?topic=/camera/camera/color/image_raw");
  const [isCameraOpen, setIsCameraOpen] = useState<boolean>(false);
  const [clickedCoordinates, setClickedCoordinates] = useState<{x: number, y: number} | null>(null);
  const [selectedArm, setSelectedArm] = useState<"left" | "right">("left");
  const [mode, setMode] = useState<"dual" | "single">("dual");

  const ros = useRef(null);
  const publisher = useRef(null);
  const listener = useRef(null);
  const realtimeControlInterval = useRef<NodeJS.Timeout | null>(null);
  const currentRealtimeCommand = useRef<{x: number, y: number, z: number, angular_z?: number}>({x: 0, y: 0, z: 0});

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
      // リアルタイム制御のintervalもクリア
      if (realtimeControlInterval.current) {
        clearInterval(realtimeControlInterval.current);
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
    // 左右アーム用のpublisherを事前に生成して再利用（パフォーマンス改善）
    const leftPub = new ROSLIB.Topic({
      ros: ros.current,
      name: "/left_arm_realtime_control",
      messageType: REALTIME_CONTROL_MESSAGE_TYPE
    });
    const rightPub = new ROSLIB.Topic({
      ros: ros.current,
      name: "/right_arm_realtime_control",
      messageType: REALTIME_CONTROL_MESSAGE_TYPE
    });
    setLeftRealtimeControlPublisher(leftPub);
    setRightRealtimeControlPublisher(rightPub);
    setRealtimeControlPublisher(null); // 互換のため保持
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

  // 変更：Poseボタンクリック時の処理（グリッドのPose 1..N用）
  const handlePoseButtonClick = (buttonNumber) => {
    if (posePublisher && connectionStatus === 'Connected') {
      // 背景色に応じたPose値を取得
      const poseValue = buttonPoseValues[backgroundColor][buttonNumber] || `${backgroundColor}_Pose1`;
      const message = new ROSLIB.Message({
        data: poseValue
      });
      posePublisher.publish(message);
      console.log(`🎯 Published pose to ${POSE_TOPIC_NAME}: "${poseValue}"`);
    } else {
      console.warn(`Cannot send pose. ROS Status: ${connectionStatus}`);
    }
  };

  // 追加：Goalボタンクリック時の処理（Goal1..Goal5専用）
  const handleGoalIndexButtonClick = (buttonNumber: number) => {
    if (posePublisher && connectionStatus === 'Connected') {
      const cap = (s: string) => s.charAt(0).toUpperCase() + s.slice(1);
      const color = cap(backgroundColor); // Red / Blue
      const arm = selectedArm; // left / right
      const goalValue = `${color}_${arm}_Goal${buttonNumber}`;
      const message = new ROSLIB.Message({ data: goalValue });
      posePublisher.publish(message);
      console.log(`🎯 Published goal to ${POSE_TOPIC_NAME}: "${goalValue}"`);
    } else {
      console.warn(`Cannot send goal. ROS Status: ${connectionStatus}`);
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

  // ゴールボタン: Field(red/blue) と Arm(left/right) に応じて /button_command にパブリッシュ
  const handleGoalButtonClick = () => {
    if (posePublisher && connectionStatus === 'Connected') {
      const capitalize = (s: string) => s.charAt(0).toUpperCase() + s.slice(1);
      const color = capitalize(backgroundColor); // Red / Blue
      const arm = selectedArm; // left / right (小文字)
      const goalValue = `${color}_${arm}_Goal`;
      const message = new ROSLIB.Message({ data: goalValue });
      posePublisher.publish(message);
      console.log(`🎯 Published goal to ${POSE_TOPIC_NAME}: "${goalValue}"`);
    } else {
      console.warn(`Cannot send goal. ROS Status: ${connectionStatus}`);
    }
  };

  const publishJointTrajectory = (position, color) => {
    let sliderValue;
    
    // ポジションに応じてslider値を設定
    switch(position) {
      case 'left':
        sliderValue = 0.193;
        break;
      case 'middle':
        sliderValue = 0.0;
        break;
      case 'right':
        sliderValue = -0.193;
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

  // 変更：アーム1初期位置
  const handleArm1Initial = () => {
    if (posePublisher && connectionStatus === 'Connected') {
      const poseValue = armPositions[backgroundColor].arm1.initial;
      const message = new ROSLIB.Message({
        data: poseValue
      });
      posePublisher.publish(message);
      console.log(`🎯 Published Arm1 Initial pose to ${POSE_TOPIC_NAME}: "${poseValue}"`);
    } else {
      console.warn(`Cannot send pose. ROS Status: ${connectionStatus}`);
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

  // 変更：アーム2初期位置
  const handleArm2Initial = () => {
    if (posePublisher && connectionStatus === 'Connected') {
      const poseValue = armPositions[backgroundColor].arm2.initial;
      const message = new ROSLIB.Message({
        data: poseValue
      });
      posePublisher.publish(message);
      console.log(`🎯 Published Arm2 Initial pose to ${POSE_TOPIC_NAME}: "${poseValue}"`);
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
    setSelectedArm(prevArm => prevArm === "left" ? "right" : "left");
  };

  const toggleCamera = () => {
    setIsCameraOpen(prev => !prev);
    if (!isCameraOpen) {
      setClickedCoordinates(null); // カメラを開く時に座標をリセット
    }
  };

  // Mode toggle function
  const toggleMode = () => {
    setMode(prevMode => prevMode === "dual" ? "single" : "dual");
  };

  // Single mode pose command function
  const sendSinglePoseCommand = (buttonNum: number) => {
    if (posePublisher && connectionStatus === 'Connected') {
      const fieldColor = backgroundColor === "red" ? "Red" : "Blue";
      const message = new ROSLIB.Message({
        data: `Single_${fieldColor}_Pose${buttonNum}`
      });
      posePublisher.publish(message);
      console.log(`Published: Single_${fieldColor}_Pose${buttonNum}`);
    }
  };

  // Single mode goal command function
  const sendSingleGoalCommand = (goalNum: number) => {
    if (posePublisher && connectionStatus === 'Connected') {
      const fieldColor = backgroundColor === "red" ? "Red" : "Blue";
      const message = new ROSLIB.Message({
        data: `Single_${fieldColor}_Goal${goalNum}`
      });
      posePublisher.publish(message);
      console.log(`Published: Single_${fieldColor}_Goal${goalNum}`);
    }
  };

  const getVisiblePoses = () => {
    if (selectedArm === "left") {
      return [1, 2, 3, 4, 5, 11, 12, 13, 14, 15, 21, 22, 23, 24, 25];
    } else {
      return [6, 7, 8, 9, 10, 16, 17, 18, 19, 20, 26, 27, 28, 29, 30];
    }
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
  const startRealtimeControl = (linear_x: number, linear_y: number, linear_z: number, angular_z: number = 0) => {
    if (ros.current && connectionStatus === 'Connected') {
      // 現在のコマンドを更新
      currentRealtimeCommand.current = { x: linear_x, y: linear_y, z: linear_z, angular_z: angular_z };

      // 既存のインターバルがあればクリア
      if (realtimeControlInterval.current) {
        clearInterval(realtimeControlInterval.current);
      }

      // 即座に最初のメッセージを送信
      publishRealtimeControlMessage(linear_x, linear_y, linear_z, angular_z);

      // 継続的にメッセージを送信（高速周期）
      realtimeControlInterval.current = setInterval(() => {
        publishRealtimeControlMessage(
          currentRealtimeCommand.current.x,
          currentRealtimeCommand.current.y,
          currentRealtimeCommand.current.z,
          currentRealtimeCommand.current.angular_z || 0
        );
      }, 20);
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
  const publishRealtimeControlMessage = (linear_x: number, linear_y: number, linear_z: number, angular_z: number = 0) => {
    if (ros.current && connectionStatus === 'Connected') {
      // アーム選択に応じて事前生成したpublisherを使用
      const publisher = selectedArm === "left" ? leftRealtimeControlPublisher : rightRealtimeControlPublisher;
      if (!publisher) return;

      const message = new ROSLIB.Message({
        linear: {
          x: linear_x,
          y: linear_y,
          z: linear_z
        },
        angular: {
          x: 0.0,
          y: 0.0,
          z: angular_z
        }
      });
      publisher.publish(message);

      // ログ出力（デバッグ用）
      console.log(`🎮 Real-time control (${selectedArm} arm): linear(${linear_x}, ${linear_y}, ${linear_z}), angular(${angular_z})`);
    }
  };

  // D-padコントローラーコンポーネント
  const DPadController = () => {
    // フィールドに応じて方向を調整
    const getLinearValues = (baseX: number, baseY: number) => {
      const multiplier = backgroundColor === "red" ? -1 : -1;
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
              const values = getLinearValues(-2.0, 0);
              startRealtimeControl(values.x, values.y, 0);
            }}
            onMouseUp={() => stopRealtimeControl()}
            onMouseLeave={() => stopRealtimeControl()}
            onTouchStart={(e) => {
              e.preventDefault();
              const values = getLinearValues(-2.0, 0);
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
              const values = getLinearValues(0, -2.0);
              startRealtimeControl(values.x, values.y, 0);
            }}
            onMouseUp={() => stopRealtimeControl()}
            onMouseLeave={() => stopRealtimeControl()}
            onTouchStart={(e) => {
              e.preventDefault();
              const values = getLinearValues(0, -2.0);
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
              const values = getLinearValues(0, 2.0);
              startRealtimeControl(values.x, values.y, 0);
            }}
            onMouseUp={() => stopRealtimeControl()}
            onMouseLeave={() => stopRealtimeControl()}
            onTouchStart={(e) => {
              e.preventDefault();
              const values = getLinearValues(0, 2.0);
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
                const values = getLinearValues(2.0, 0);
                startRealtimeControl(values.x, values.y, 0);
              }}
              onMouseUp={() => stopRealtimeControl()}
              onMouseLeave={() => stopRealtimeControl()}
              onTouchStart={(e) => {
                e.preventDefault();
                const values = getLinearValues(2.0, 0);
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
          onMouseDown={() => startRealtimeControl(0, 0, 0, 4.0)}
          onMouseUp={() => stopRealtimeControl()}
          onMouseLeave={() => stopRealtimeControl()}
          onTouchStart={(e) => {
            e.preventDefault();
            startRealtimeControl(0, 0, 0, 4.0);
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
          onMouseDown={() => startRealtimeControl(0, 0, 0, -4.0)}
          onMouseUp={() => stopRealtimeControl()}
          onMouseLeave={() => stopRealtimeControl()}
          onTouchStart={(e) => {
            e.preventDefault();
            startRealtimeControl(0, 0, 0, -4.0);
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
    </div>
  );

  const CameraView = () => (
    <div className="camera-view-container-fullscreen">
      <div className="camera-image-wrapper">
        <img 
          src="http://192.168.10.151:8080/stream?topic=/camera/camera/color/image_raw" 
          alt="Camera Feed" 
          className={`camera-image ${selectedArm === "left" ? "camera-image-left" : "camera-image-right"}`}
          onClick={handleImageClick}
          onError={(e) => {
            console.log('カメラ映像読み込みエラー');
            const img = e.target as HTMLImageElement;
            setTimeout(() => {
              img.src = "http://192.168.10.151:8080/stream?topic=/camera/camera/color/image_raw";
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
            <div className="up-down-buttons vertical">
              <button
                className="up-down-button up-button"
                onMouseDown={() => startRealtimeControl(0, 0, 1.0)}
                onMouseUp={() => stopRealtimeControl()}
                onMouseLeave={() => stopRealtimeControl()}
                onTouchStart={(e) => {
                  e.preventDefault();
                  startRealtimeControl(0, 0, 1.0);
                }}
                onTouchEnd={(e) => {
                  e.preventDefault();
                  stopRealtimeControl();
                }}
                disabled={connectionStatus !== 'Connected'}
              >
                ⬆️ UP
              </button>
              <button
                className="up-down-button down-button"
                onMouseDown={() => startRealtimeControl(0, 0, -1.0)}
                onMouseUp={() => stopRealtimeControl()}
                onMouseLeave={() => stopRealtimeControl()}
                onTouchStart={(e) => {
                  e.preventDefault();
                  startRealtimeControl(0, 0, -1.0);
                }}
                onTouchEnd={(e) => {
                  e.preventDefault();
                  stopRealtimeControl();
                }}
                disabled={connectionStatus !== 'Connected'}
              >
                ⬇️ DOWN
              </button>
            </div>
          </div>
        </div>
      </div>

      {/* フルスクリーン時のゴールボタン（右下に表示） */}
      <div className="goal-buttons-area goal-buttons-area-fs">
        <div className="goal-buttons-container horizontal">
          <button 
            className="goal-button small-goal-button"
            onClick={() => handleGoalIndexButtonClick(1)}
            disabled={connectionStatus !== 'Connected'}
          >
            Goal 1
          </button>
          <button 
            className="goal-button small-goal-button"
            onClick={() => handleGoalIndexButtonClick(2)}
            disabled={connectionStatus !== 'Connected'}
          >
            Goal 2
          </button>
          <button 
            className="goal-button small-goal-button"
            onClick={() => handleGoalIndexButtonClick(3)}
            disabled={connectionStatus !== 'Connected'}
          >
            Goal 3
          </button>
          <button 
            className="goal-button small-goal-button"
            onClick={() => handleGoalIndexButtonClick(4)}
            disabled={connectionStatus !== 'Connected'}
          >
            Goal 4
          </button>
          <button 
            className="goal-button small-goal-button"
            onClick={() => handleGoalIndexButtonClick(5)}
            disabled={connectionStatus !== 'Connected'}
          >
            Goal 5
          </button>
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
          src="http://192.168.100.3:8080/stream?topic=/camera/camera/color/image_raw" 
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
              img.src = "http://192.168.100.3:8080/stream?topic=/camera/camera/color/image_raw";
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
      className="app-container no-select"
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
            🦾 Arm: {selectedArm === "left" ? "左" : "右"}
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
            className={`toggle-button mode-toggle`}
            onClick={toggleMode}
          >
            🔢 {mode === "dual" ? "dual" : "single"}
          </button>
        </div>
      </header>
      
      <div className="pose-grid-container">
        {mode === "single" ? (
          // Single mode: 3x10 Pose button grid + Goal buttons
          <div className="single-mode-container">
            <div className="pose-grid-single">
              {Array.from({ length: 30 }, (_, index) => (
                <button
                  key={index}
                  className="pose-button-single"
                  onClick={() => sendSinglePoseCommand(index + 1)}
                  disabled={connectionStatus !== 'Connected'}
                >
                  Pose{index + 1}
                </button>
              ))}
            </div>
            {/* Goal buttons for single mode - placed below pose buttons */}
            <div className="goal-buttons-container-single">
              {Array.from({ length: 5 }, (_, index) => (
                <button
                  key={index}
                  className="goal-button-single"
                  onClick={() => sendSingleGoalCommand(index + 1)}
                  disabled={connectionStatus !== 'Connected'}
                >
                  Goal{index + 1}
                </button>
              ))}
            </div>
          </div>
        ) : isCameraOpen ? (
          <div className="camera-view-container">
            <div className="camera-image-wrapper">
              {/* カメラ画像（Poseエリアの代わりに表示） */}
              <img 
                src={cameraImageUrl}
                alt="Camera Feed View" 
                className={selectedArm === "left" ? "camera-image-left" : "camera-image-right"}
                onClick={handleImageClick}
                onError={(e) => {
                  console.log('カメラ映像読み込みエラー (通常ビュー)');
                  const img = e.target as HTMLImageElement;
                  setTimeout(() => {
                    img.src = cameraImageUrl;
                  }, 3000);
                }}
                style={{ cursor: 'crosshair' }}
              />
              {clickedCoordinates && (
                <div className="coordinate-display-inline">
                  ({clickedCoordinates.x}, {clickedCoordinates.y})
                </div>
              )}
            </div>
            <div className="right-panel">
              <div className="side-controls-expanded">
                <div className="controller-section-horizontal">
                  <div className="left-controller-section">
                    <YawController />
                    <DPadController />
                  </div>
                  <div className="right-controller-section">
                    <div className="up-down-buttons vertical">
                      <button
                        className="up-down-button up-button"
                        onMouseDown={() => startRealtimeControl(0, 0, 1.0)}
                        onMouseUp={() => stopRealtimeControl()}
                        onMouseLeave={() => stopRealtimeControl()}
                        onTouchStart={(e) => {
                          e.preventDefault();
                          startRealtimeControl(0, 0, 1.0);
                        }}
                        onTouchEnd={(e) => {
                          e.preventDefault();
                          stopRealtimeControl();
                        }}
                        disabled={connectionStatus !== 'Connected'}
                      >
                        ⬆️ UP
                      </button>
                      <button
                        className="up-down-button down-button"
                        onMouseDown={() => startRealtimeControl(0, 0, -1.0)}
                        onMouseUp={() => stopRealtimeControl()}
                        onMouseLeave={() => stopRealtimeControl()}
                        onTouchStart={(e) => {
                          e.preventDefault();
                          startRealtimeControl(0, 0, -1.0);
                        }}
                        onTouchEnd={(e) => {
                          e.preventDefault();
                          stopRealtimeControl();
                        }}
                        disabled={connectionStatus !== 'Connected'}
                      >
                        ⬇️ DOWN
                      </button>
                    </div>
                    <div className="arrow-buttons">
                      <button
                        className="arrow-button left-arrow"
                        onMouseDown={() => startRealtimeControl(0, 1.0, 0, 3.0)}
                        onMouseUp={() => stopRealtimeControl()}
                        onMouseLeave={() => stopRealtimeControl()}
                        onTouchStart={(e) => {
                          e.preventDefault();
                          startRealtimeControl(0, 1.0, 0, 1.0);
                        }}
                        onTouchEnd={(e) => {
                          e.preventDefault();
                          stopRealtimeControl();
                        }}
                        disabled={connectionStatus !== 'Connected'}
                      >
                        ⬅
                      </button>
                      <button
                        className="arrow-button right-arrow"
                        onMouseDown={() => startRealtimeControl(0, -1.0, 0, -3.0)}
                        onMouseUp={() => stopRealtimeControl()}
                        onMouseLeave={() => stopRealtimeControl()}
                        onTouchStart={(e) => {
                          e.preventDefault();
                          startRealtimeControl(0, -1.0, 0, -1.0);
                        }}
                        onTouchEnd={(e) => {
                          e.preventDefault();
                          stopRealtimeControl();
                        }}
                        disabled={connectionStatus !== 'Connected'}
                      >
                        ➡
                      </button>
                    </div>
                  </div>
                </div>
              </div>
              <div className="goal-buttons-area">
                <div className="goal-buttons-container horizontal">
          <button 
            className="goal-button small-goal-button"
            onClick={() => handleGoalIndexButtonClick(1)}
            disabled={connectionStatus !== 'Connected'}
          >
            Goal 1
          </button>
          <button 
            className="goal-button small-goal-button"
            onClick={() => handleGoalIndexButtonClick(2)}
            disabled={connectionStatus !== 'Connected'}
          >
            Goal 2
          </button>
          <button 
            className="goal-button small-goal-button"
            onClick={() => handleGoalIndexButtonClick(3)}
            disabled={connectionStatus !== 'Connected'}
          >
            Goal 3
          </button>
          <button 
            className="goal-button small-goal-button"
            onClick={() => handleGoalIndexButtonClick(4)}
            disabled={connectionStatus !== 'Connected'}
          >
            Goal 4
          </button>
          <button 
            className="goal-button small-goal-button"
            onClick={() => handleGoalIndexButtonClick(5)}
            disabled={connectionStatus !== 'Connected'}
          >
            Goal 5
          </button>
                </div>
              </div>
            </div>
          </div>
        ) : (
          <div className="camera-view-container">
            <div className="camera-image-wrapper">
              <div className="pose-grid">
                {getVisiblePoses().map((buttonNumber) => (
                  <GridButton 
                    key={buttonNumber} 
                    buttonNumber={buttonNumber}
                  />
                ))}
              </div>
            </div>
            <div className="right-panel">
              <div className="side-controls-expanded">
                <div className="controller-section-horizontal">
                  <div className="left-controller-section">
                    <YawController />
                    <DPadController />
                  </div>
                  <div className="right-controller-section">
                    <div className="up-down-buttons vertical">
                      <button
                        className="up-down-button up-button"
                        onMouseDown={() => startRealtimeControl(0, 0, 1.0)}
                        onMouseUp={() => stopRealtimeControl()}
                        onMouseLeave={() => stopRealtimeControl()}
                        onTouchStart={(e) => {
                          e.preventDefault();
                          startRealtimeControl(0, 0, 1.0);
                        }}
                        onTouchEnd={(e) => {
                          e.preventDefault();
                          stopRealtimeControl();
                        }}
                        disabled={connectionStatus !== 'Connected'}
                      >
                        ⬆️ UP
                      </button>
                      <button
                        className="up-down-button down-button"
                        onMouseDown={() => startRealtimeControl(0, 0, -1.0)}
                        onMouseUp={() => stopRealtimeControl()}
                        onMouseLeave={() => stopRealtimeControl()}
                        onTouchStart={(e) => {
                          e.preventDefault();
                          startRealtimeControl(0, 0, -1.0);
                        }}
                        onTouchEnd={(e) => {
                          e.preventDefault();
                          stopRealtimeControl();
                        }}
                        disabled={connectionStatus !== 'Connected'}
                      >
                        ⬇️ DOWN
                      </button>
                    </div>
                    <div className="arrow-buttons">
                      <button
                        className="arrow-button left-arrow"
                        onMouseDown={() => startRealtimeControl(0, 1.0, 0, 1.0)}
                        onMouseUp={() => stopRealtimeControl()}
                        onMouseLeave={() => stopRealtimeControl()}
                        onTouchStart={(e) => {
                          e.preventDefault();
                          startRealtimeControl(0, 1.0, 0, 1.0);
                        }}
                        onTouchEnd={(e) => {
                          e.preventDefault();
                          stopRealtimeControl();
                        }}
                        disabled={connectionStatus !== 'Connected'}
                      >
                        ⬅
                      </button>
                      <button
                        className="arrow-button right-arrow"
                        onMouseDown={() => startRealtimeControl(0, -1.0, 0, -1.0)}
                        onMouseUp={() => stopRealtimeControl()}
                        onMouseLeave={() => stopRealtimeControl()}
                        onTouchStart={(e) => {
                          e.preventDefault();
                          startRealtimeControl(0, -1.0, 0, -1.0);
                        }}
                        onTouchEnd={(e) => {
                          e.preventDefault();
                          stopRealtimeControl();
                        }}
                        disabled={connectionStatus !== 'Connected'}
                      >
                        ➡
                      </button>
                    </div>
                  </div>
                </div>
              </div>
              <div className="goal-buttons-area">
                <div className="goal-buttons-container horizontal">
          <button 
            className="goal-button small-goal-button"
            onClick={() => handleGoalIndexButtonClick(1)}
            disabled={connectionStatus !== 'Connected'}
          >
            Goal 1
          </button>
          <button 
            className="goal-button small-goal-button"
            onClick={() => handleGoalIndexButtonClick(2)}
            disabled={connectionStatus !== 'Connected'}
          >
            Goal 2
          </button>
          <button 
            className="goal-button small-goal-button"
            onClick={() => handleGoalIndexButtonClick(3)}
            disabled={connectionStatus !== 'Connected'}
          >
            Goal 3
          </button>
          <button 
            className="goal-button small-goal-button"
            onClick={() => handleGoalIndexButtonClick(4)}
            disabled={connectionStatus !== 'Connected'}
          >
            Goal 4
          </button>
          <button 
            className="goal-button small-goal-button"
            onClick={() => handleGoalIndexButtonClick(5)}
            disabled={connectionStatus !== 'Connected'}
          >
            Goal 5
          </button>
                </div>
              </div>
            </div>
          </div>
        )}
      </div>

      {/* 旧ゴールボタンエリアは削除（中央のカメラ/グリッドコンテナ内に移動） */}

      <div className="bottom-control-area">
        {selectedArm === "left" && (
          <div className="arm-controls single-arm">
            {/* ゴールボタン（掴むの左） */}
            <button 
              className="arm-button goal-pose-button"
              onClick={handleGoalButtonClick}
              disabled={connectionStatus !== 'Connected'}
            >
              🎯 Goal
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
        
        {selectedArm === "right" && (
          <div className="arm-controls single-arm">
            {/* ゴールボタン（掴むの左） */}
            <button 
              className="arm-button goal-pose-button"
              onClick={handleGoalButtonClick}
              disabled={connectionStatus !== 'Connected'}
            >
              🎯 Goal
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
