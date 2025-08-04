#!/usr/bin/env python3
import sys
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QPushButton,
    QVBoxLayout, QGridLayout, QHBoxLayout, QSizePolicy
)
from PyQt6.QtCore import QCoreApplication

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # ウィンドウの基本的な設定
        self.setWindowTitle("Custom Robot Controller")
        self.setGeometry(100, 100, 800, 600)

        # === メインレイアウトの作成 ===
        main_layout = QVBoxLayout()

        # --- 1. 上部グリッドレイアウトの作成 ---
        grid_layout = QGridLayout()
        for row in range(4):
            for col in range(5):
                button_text = f"Button {row*5 + col + 1}"
                button = QPushButton(button_text)
                button.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
                button.clicked.connect(
                    lambda checked, text=button_text: self.on_button_clicked(text)
                )
                grid_layout.addWidget(button, row, col)
        
        # --- 2. 下部ボタンレイアウトの作成 ---
        bottom_layout = QHBoxLayout()

        # 「ゴール」ボタン
        goal_button = QPushButton("ゴール") # [cite: 1]
        goal_button.clicked.connect(lambda: self.on_button_clicked("ゴール"))
        
        # 「初期位置」ボタン
        init_pos_1_button = QPushButton("アーム1\n初期位置") # [cite: 2]
        # ▼▼▼ ここのテキストを修正 ▼▼▼
        init_pos_2_button = QPushButton("アーム2\n初期位置") # 
        
        # ボタンのスタイルを設定して楕円形に近づける
        oval_button_style = """
            QPushButton {
                border: 2px solid #8f8f91;
                border-radius: 30px; /* 角の丸みを半径で指定 */
                background-color: #f6f6f6;
                padding: 10px;
                min-height: 60px; /* 最小の高さを設定 */
            }
            QPushButton:pressed {
                background-color: #cde8ff;
            }
        """
        init_pos_1_button.setStyleSheet(oval_button_style)
        init_pos_2_button.setStyleSheet(oval_button_style)

        init_pos_1_button.clicked.connect(lambda: self.on_button_clicked("アーム1 初期位置"))
        # ▼▼▼ ここで渡すテキストも修正 ▼▼▼
        init_pos_2_button.clicked.connect(lambda: self.on_button_clicked("アーム2 初期位置"))
        
        # レイアウトにウィジェットを追加
        bottom_layout.addWidget(goal_button)
        bottom_layout.addStretch(1) 
        bottom_layout.addWidget(init_pos_1_button)
        bottom_layout.addWidget(init_pos_2_button)
        bottom_layout.addStretch(1) 
        
        # === 全体を結合 ===
        main_layout.addLayout(grid_layout, stretch=3) 
        main_layout.addLayout(bottom_layout, stretch=1)

        # メインウィジェットとして設定
        main_widget = QWidget()
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)
        

    def on_button_clicked(self, button_text):
        """ボタンがクリックされたときに呼ばれる関数"""
        print(f"Button '{button_text}' was clicked.")

def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()