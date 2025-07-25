import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QComboBox, QLabel

class MyWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Dropdown Example")
        self.setGeometry(100, 100, 300, 100)

        # Variable to store current selection
        self.current_value = None

        # Create layout
        layout = QVBoxLayout()

        # Create dropdown
        self.combo = QComboBox()
        self.combo.addItems(["Option 1", "Option 2", "Option 3"])
        self.combo.currentTextChanged.connect(self.on_combobox_changed)

        # Label to show current value
        self.label = QLabel("Current selection: None")

        layout.addWidget(self.combo)
        layout.addWidget(self.label)
        self.setLayout(layout)

    def on_combobox_changed(self, text):
        self.current_value = text  # update the variable
        print(f"Current selection: {self.current_value}")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MyWindow()
    window.show()
    sys.exit(app.exec_())
