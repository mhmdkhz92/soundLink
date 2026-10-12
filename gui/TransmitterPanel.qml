import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic

ScrollView {
    id: panel
    property bool compact: false
    contentWidth: availableWidth
    clip: true
    ColumnLayout {
        width: panel.availableWidth
        spacing: 18
        Item { Layout.preferredHeight: 6 }
        ColumnLayout {
            Layout.fillWidth: true
            Layout.leftMargin: panel.compact ? 14 : 24
            Layout.rightMargin: panel.compact ? 14 : 24
            spacing: 6
            Label { text: "SOUNDLINK"; color: "#39d6b2"; font.pixelSize: 12; font.letterSpacing: 3 }
            Label { text: "Transmitter"; font.pixelSize: panel.compact ? 24 : 30; font.weight: Font.DemiBold }
            Label { text: transmitter.status; color: transmitter.running ? "#39d6b2" : "#91a5ba"; Layout.fillWidth: true; wrapMode: Text.WordWrap }
        }
        Rectangle {
            Layout.fillWidth: true
            Layout.leftMargin: panel.compact ? 14 : 24
            Layout.rightMargin: panel.compact ? 14 : 24
            implicitHeight: settings.implicitHeight + 32
            color: "#131f2e"; border.color: "#26364a"; radius: 14
            GridLayout {
                id: settings
                anchors.fill: parent; anchors.margins: 16
                columns: panel.compact ? 2 : 4
                columnSpacing: 18; rowSpacing: 12
                ColumnLayout {
                    Layout.fillWidth: true
                    Label { text: "OUTPUT"; color: "#91a5ba"; font.pixelSize: 12 }
                    Label { text: "Default speaker"; font.pixelSize: 13 }
                    Label { text: "Mono · 48 kHz"; color: "#91a5ba"; font.pixelSize: 12 }
                }
                ColumnLayout {
                    Layout.fillWidth: true
                    Label { text: "BANDWIDTH · kHz"; color: "#91a5ba"; font.pixelSize: 12 }
                    ComboBox { id: bw; Layout.fillWidth: true; Layout.minimumHeight: 44; model: ["1.4", "3", "5", "10"]; currentIndex: 2; enabled: !transmitter.running }
                }
                ColumnLayout {
                    Layout.fillWidth: true
                    Label { text: "CODE RATE"; color: "#91a5ba"; font.pixelSize: 12 }
                    ComboBox { id: rate; Layout.fillWidth: true; Layout.minimumHeight: 44; model: ["1/2", "2/3", "3/4", "5/6", "7/8"]; enabled: !transmitter.running }
                }
                ColumnLayout {
                    Layout.fillWidth: true
                    Label { text: "CENTER · kHz"; color: "#91a5ba"; font.pixelSize: 12 }
                    SpinBox { id: fc; Layout.fillWidth: true; Layout.minimumHeight: 44; from: 2; to: 18; value: 5; editable: true; enabled: !transmitter.running }
                }
                Button {
                    Layout.columnSpan: settings.columns
                    Layout.fillWidth: true; Layout.minimumHeight: 48
                    text: transmitter.running ? "Stop transmitter" : "Start transmitter"
                    font.weight: Font.DemiBold
                    onClicked: {
                        if (transmitter.running) transmitter.stop()
                        else { receiver.stop(); transmitter.start(Number(bw.currentText), rate.currentText, fc.value) }
                    }
                    background: Rectangle { radius: 8; color: parent.down ? "#239b83" : "#39d6b2" }
                    contentItem: Text { text: parent.text; font: parent.font; color: "#09231e"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                }
            }
        }
        Rectangle {
            Layout.fillWidth: true
            Layout.leftMargin: panel.compact ? 14 : 24
            Layout.rightMargin: panel.compact ? 14 : 24
            implicitHeight: details.implicitHeight + 36
            color: "#131f2e"; border.color: "#26364a"; radius: 14
            ColumnLayout {
                id: details
                anchors.fill: parent; anchors.margins: 18
                spacing: 12
                Label { text: "Continuous test signal"; font.pixelSize: 18; font.weight: Font.DemiBold }
                Label { text: "Sends the same PRBS test sequence as your command-line transmitter. Match these settings on the receiver to measure BER."; color: "#91a5ba"; Layout.fillWidth: true; wrapMode: Text.WordWrap }
                Label { text: "Use the phone’s volume buttons to adjust output level."; color: "#91a5ba"; Layout.fillWidth: true; wrapMode: Text.WordWrap }
                Label { text: "Underrun frames  " + transmitter.underrunFrames; color: transmitter.underrunFrames > 0 ? "#ffc078" : "#91a5ba" }
                Label { text: "Switching Receiver / Transmitter stops the current session."; color: "#91a5ba"; Layout.fillWidth: true; wrapMode: Text.WordWrap }
            }
        }
    }
}
