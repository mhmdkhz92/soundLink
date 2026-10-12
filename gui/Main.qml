import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic

ApplicationWindow {
    id: window
    width: 1180
    height: 820
    minimumWidth: 320
    minimumHeight: 320
    visible: true
    title: "SoundLink"
    color: "#0c1420"
    font.family: "Helvetica Neue"
    palette.windowText: "#e4edf6"
    palette.text: "#e4edf6"
    palette.buttonText: "#e4edf6"
    palette.button: "#223248"
    palette.base: "#172538"
    palette.highlight: "#39d6b2"
    property bool compact: width < 600
    property int pageMargin: compact ? 14 : 24
    property color muted: "#91a5ba"
    property color accent: "#39d6b2"
    property bool starting: false
    property bool busy: receiver.running || starting
    property real selectedBandwidth: Number(bandwidth.currentText)
    property real selectedCenter: center.value

    function berText(value) {
        return value < 0 ? "—" : value === 0 ? "0" : value.toExponential(2)
    }

    Connections {
        target: receiver
        function onUpdated() {
            window.starting = receiver.status === "Waiting for microphone permission"
            if (modeTabs.currentIndex === 0) {
                if (plotTabs.currentIndex === 0) spectrumPlot.requestPaint()
                else if (plotTabs.currentIndex === 1) constellationPlot.requestPaint()
            }
        }
    }
    onClosing: { receiver.stop(); transmitter.stop(); }
    function selectDisplay() {
        receiver.setDisplay(modeTabs.currentIndex === 0 ? plotTabs.currentIndex : -1)
    }
    Component.onCompleted: selectDisplay()
    header: TabBar {
        id: modeTabs
        spacing: 6
        background: Rectangle { color: "#0c1420" }
        onCurrentIndexChanged: {
            if (currentIndex === 0) transmitter.stop()
            else { receiver.stop(); window.starting = false }
            window.selectDisplay()
        }
        NavigationTab { text: "Receiver"; implicitHeight: 48 }
        NavigationTab { text: "Transmitter"; implicitHeight: 48 }
    }
    TransmitterPanel {
        anchors.fill: parent
        visible: modeTabs.currentIndex === 1
        compact: window.compact
    }

    component NavigationTab: TabButton {
        implicitHeight: 48
        background: Rectangle {
            color: parent.checked ? "#153a36" : "#172538"
            radius: 8
            border.color: parent.checked ? "#39d6b2" : "#26364a"
        }
        contentItem: Text {
            text: parent.text
            font: parent.font
            color: parent.checked ? "#39d6b2" : "#b6c5d4"
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
        }
    }
    component Card: Rectangle {
        color: "#131f2e"
        border.color: "#26364a"
        radius: 14
    }
    component Caption: Label {
        color: window.muted
        font.pixelSize: 12
    }
    component Metric: Card {
        property string caption
        property string value
        property string detail
        implicitHeight: 122
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 18
            spacing: 5
            Caption { text: caption; font.letterSpacing: 1 }
            Label { text: value; font.pixelSize: window.compact ? 24 : 30; color: "#e4edf6" }
            Caption { text: detail; Layout.fillWidth: true; elide: Text.ElideRight }
        }
    }

    ScrollView {
        id: scroll
        visible: modeTabs.currentIndex === 0
        anchors.fill: parent
        contentWidth: availableWidth
        clip: true
        ColumnLayout {
            width: scroll.availableWidth
            spacing: 18
            // Margins are supplied by the outer content items.
            Item { Layout.preferredHeight: 6 }
            GridLayout {
                columns: window.compact ? 1 : 3
                Layout.fillWidth: true
                Layout.leftMargin: window.pageMargin
                Layout.rightMargin: window.pageMargin
                ColumnLayout {
                    spacing: 3
                    Caption { text: "SOUNDLINK"; color: window.accent; font.letterSpacing: 3 }
                    Label { text: "Receiver"; font.pixelSize: window.compact ? 24 : 30; font.weight: Font.DemiBold }
                }
                Item { Layout.fillWidth: true; visible: !window.compact }
                Rectangle {
                    Layout.maximumWidth: scroll.availableWidth - 2 * window.pageMargin
                    implicitWidth: statusLabel.implicitWidth + 34
                    implicitHeight: 34
                    radius: 17
                    color: receiver.running ? "#153a36" : "#223044"
                    Label {
                        id: statusLabel
                        anchors.fill: parent
                        anchors.leftMargin: 17
                        anchors.rightMargin: 17
                        verticalAlignment: Text.AlignVCenter
                        elide: Text.ElideRight
                        text: receiver.status
                        color: receiver.running ? window.accent : "#b6c5d4"
                        font.pixelSize: 12
                    }
                }
            }

            Card {
                Layout.fillWidth: true
                Layout.leftMargin: window.pageMargin
                Layout.rightMargin: window.pageMargin
                implicitHeight: controls.implicitHeight + 32
                GridLayout {
                    id: controls
                    anchors.fill: parent
                    anchors.margins: 16
                    columns: window.width < 800 ? 2 : 5
                    columnSpacing: 18
                    rowSpacing: 12
                    ColumnLayout {
                        Layout.fillWidth: true
                        Caption { text: "INPUT" }
                        Label { text: "Default microphone"; font.pixelSize: 13 }
                        Caption { text: "Mono · 48 kHz" }
                    }
                    ColumnLayout {
                        Layout.fillWidth: true
                        Caption { text: "BANDWIDTH · kHz" }
                        ComboBox {
                            id: bandwidth
                            Layout.minimumHeight: 44
                            Layout.fillWidth: true
                            model: ["1.4", "3", "5", "10"]
                            currentIndex: 2
                            enabled: !window.busy
                            onCurrentTextChanged: spectrumPlot.requestPaint()
                        }
                    }
                    ColumnLayout {
                        Layout.fillWidth: true
                        Caption { text: "CODE RATE" }
                        ComboBox {
                            id: codeRate
                            Layout.minimumHeight: 44
                            Layout.fillWidth: true
                            model: ["1/2", "2/3", "3/4", "5/6", "7/8"]
                            enabled: !window.busy
                        }
                    }
                    ColumnLayout {
                        Layout.fillWidth: true
                        Caption { text: "CENTER · kHz" }
                        SpinBox {
                            id: center
                            Layout.minimumHeight: 44
                            Layout.fillWidth: true
                            from: 2; to: 18; value: 5
                            editable: true
                            enabled: !window.busy
                            onValueChanged: spectrumPlot.requestPaint()
                        }
                    }
                    Button {
                        Layout.columnSpan: controls.columns === 2 ? 2 : 1
                        Layout.minimumHeight: 48
                        Layout.fillWidth: true
                        Layout.alignment: Qt.AlignBottom
                        text: window.busy ? "Stop receiver" : "Start receiver"
                        font.weight: Font.DemiBold
                        onClicked: {
                            if (window.busy) {
                                receiver.stop()
                                window.starting = false
                            } else {
                                receiver.start(window.selectedBandwidth, codeRate.currentText, center.value)
                            }
                        }
                        background: Rectangle {
                            radius: 8
                            color: parent.down ? "#239b83" : window.accent
                            implicitHeight: 40
                        }
                        contentItem: Text {
                            text: parent.text
                            font: parent.font
                            color: "#09231e"
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                        }
                    }
                }
            }

            GridLayout {
                Layout.fillWidth: true
                Layout.leftMargin: window.pageMargin
                Layout.rightMargin: window.pageMargin
                columns: window.width < 800 ? 2 : 4
                columnSpacing: 14
                rowSpacing: 14
                Metric {
                    Layout.fillWidth: true
                    caption: "TOTAL BER"
                    value: window.berText(receiver.totalBer)
                    detail: "Valid compared bits this session"
                }
                Metric {
                    Layout.fillWidth: true
                    caption: "RECENT BER"
                    value: window.berText(receiver.recentBer)
                    detail: "Rolling window · approximately 1 s"
                }
                Metric {
                    Layout.fillWidth: true
                    caption: "COMPARED BITS"
                    value: Number(receiver.comparedBits).toLocaleString(Qt.locale(), 'f', 0)
                    detail: Number(receiver.errorBits).toLocaleString(Qt.locale(), 'f', 0) + " bit errors"
                }
                Metric {
                    Layout.fillWidth: true
                    caption: "RECEIVED RATE"
                    value: receiver.dataRate < 0 ? "—" : (receiver.dataRate / 1000).toFixed(2)
                    detail: "kbit/s · decoded payload while locked"
                }
            }

            ColumnLayout {
                Layout.fillWidth: true
                Layout.leftMargin: window.pageMargin
                Layout.rightMargin: window.pageMargin
                spacing: 10
                TabBar {
                    id: plotTabs
                    spacing: 6
                    background: Rectangle { color: "#0c1420" }
                    Layout.fillWidth: true
                    onCurrentIndexChanged: window.selectDisplay()
                    NavigationTab { text: "Spectrum"; implicitHeight: 44 }
                    NavigationTab { text: "Constellation"; implicitHeight: 44 }
                    NavigationTab { text: "SNR / RB"; implicitHeight: 44 }
                }
                StackLayout {
                    Layout.fillWidth: true
                    Layout.preferredHeight: window.compact ? 340 : 400
                    currentIndex: plotTabs.currentIndex
                    Card {
                    Layout.fillWidth: true
                    Layout.minimumWidth: 0
                    
                    
                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 18
                        RowLayout {
                            Layout.fillWidth: true
                            Label { text: "Spectrum"; font.pixelSize: 17; font.weight: Font.DemiBold }
                            Item { Layout.fillWidth: true }
                            Caption { text: "0–24 kHz · dBFS" }
                        }
                        Canvas {
                            id: spectrumPlot
                            onVisibleChanged: if (visible) requestPaint()
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            onWidthChanged: requestPaint()
                            onHeightChanged: requestPaint()
                            onPaint: {
                                if (!visible || modeTabs.currentIndex !== 0) return
                                const ctx = getContext("2d")
                                ctx.clearRect(0, 0, width, height)
                                const left = 38, top = 12, right = width - 12, bottom = height - 28
                                const w = right - left, h = bottom - top
                                if (w <= 0 || h <= 0) return
                                const x = function(khz) { return left + khz / 24 * w }
                                const y = function(db) { return top + Math.max(0, Math.min(120, -db)) / 120 * h }
                                const low = Math.max(0, window.selectedCenter - window.selectedBandwidth / 2)
                                const high = Math.min(24, window.selectedCenter + window.selectedBandwidth / 2)
                                ctx.fillStyle = "#183d3a"
                                ctx.fillRect(x(low), top, x(high) - x(low), h)
                                ctx.font = "10px sans-serif"
                                ctx.lineWidth = 1
                                for (let db = 0; db >= -120; db -= 30) {
                                    ctx.strokeStyle = "#293b4e"
                                    ctx.beginPath(); ctx.moveTo(left, y(db)); ctx.lineTo(right, y(db)); ctx.stroke()
                                    ctx.fillStyle = "#91a5ba"; ctx.textAlign = "right"
                                    ctx.fillText(String(db), left - 7, y(db) + 4)
                                }
                                for (let khz = 0; khz <= 24; khz += 6) {
                                    ctx.strokeStyle = "#293b4e"
                                    ctx.beginPath(); ctx.moveTo(x(khz), top); ctx.lineTo(x(khz), bottom); ctx.stroke()
                                    ctx.fillStyle = "#91a5ba"; ctx.textAlign = "center"
                                    ctx.fillText(String(khz), x(khz), bottom + 18)
                                }
                                ctx.strokeStyle = "#39d6b2"
                                ctx.setLineDash([4, 4])
                                ctx.beginPath(); ctx.moveTo(x(window.selectedCenter), top); ctx.lineTo(x(window.selectedCenter), bottom); ctx.stroke()
                                ctx.setLineDash([])
                                const bins = receiver.spectrum
                                if (bins.length) {
                                    ctx.strokeStyle = "#6ce8cc"; ctx.lineWidth = 1.5; ctx.beginPath()
                                    for (let i = 0; i < bins.length; ++i) {
                                        const px = x(bins[i].frequency / 1000), py = y(bins[i].db)
                                        if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py)
                                    }
                                    ctx.stroke()
                                } else {
                                    ctx.fillStyle = "#91a5ba"; ctx.textAlign = "center"; ctx.font = "12px sans-serif"
                                    ctx.fillText("Start the receiver to view microphone audio", left + w / 2, top + h / 2)
                                }
                            }
                        }
                        Caption { text: "Shaded band: selected bandwidth · dashed line: center frequency"; font.pixelSize: 10; Layout.fillWidth: true; wrapMode: Text.WordWrap }
                    }
                }
                    Card {
                    Layout.fillWidth: true
                    Layout.minimumWidth: 0
                    
                    
                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 18
                        RowLayout {
                            Layout.fillWidth: true
                            Label { text: "Constellation"; font.pixelSize: 17; font.weight: Font.DemiBold }
                            Item { Layout.fillWidth: true }
                            Caption { text: "Equalized · QPSK" }
                        }
                        Canvas {
                            id: constellationPlot
                            onVisibleChanged: if (visible) requestPaint()
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            onWidthChanged: requestPaint()
                            onHeightChanged: requestPaint()
                            onPaint: {
                                if (!visible || modeTabs.currentIndex !== 0) return
                                const ctx = getContext("2d")
                                ctx.clearRect(0, 0, width, height)
                                const size = Math.min(width - 40, height - 30)
                                if (size <= 0) return
                                const cx = width / 2, cy = (height - 20) / 2, scale = size / 3
                                ctx.font = "10px sans-serif"
                                for (let v = -1; v <= 1; v += 0.5) {
                                    ctx.strokeStyle = v === 0 ? "#52657a" : "#26384b"
                                    ctx.beginPath()
                                    ctx.moveTo(cx + v * scale, cy - size / 2); ctx.lineTo(cx + v * scale, cy + size / 2)
                                    ctx.moveTo(cx - size / 2, cy - v * scale); ctx.lineTo(cx + size / 2, cy - v * scale)
                                    ctx.stroke()
                                }
                                ctx.save()
                                ctx.beginPath(); ctx.rect(cx - size / 2, cy - size / 2, size, size); ctx.clip()
                                // Unit-energy QPSK reference points.
                                ctx.strokeStyle = "#d8e4f0"; ctx.lineWidth = 1.5
                                for (const re of [-Math.SQRT1_2, Math.SQRT1_2]) {
                                    for (const im of [-Math.SQRT1_2, Math.SQRT1_2]) {
                                        const px = cx + re * scale, py = cy - im * scale
                                        ctx.beginPath(); ctx.arc(px, py, 6, 0, 2 * Math.PI); ctx.stroke()
                                    }
                                }
                                ctx.fillStyle = "#39d6b2"; ctx.globalAlpha = 0.55
                                const points = receiver.constellation
                                for (let i = 0; i < points.length; ++i) {
                                    ctx.beginPath(); ctx.arc(cx + points[i].x * scale, cy - points[i].y * scale, 1.7, 0, 2 * Math.PI); ctx.fill()
                                }
                                ctx.restore()
                                ctx.fillStyle = "#91a5ba"; ctx.textAlign = "center"
                                ctx.fillText("In-phase (I)", cx, height - 2)
                                if (!points.length) {
                                    ctx.font = "12px sans-serif"
                                    ctx.fillText("Waiting for decoded symbols", cx, cy)
                                }
                            }
                        }
                        Caption { text: "Outlined circles: ideal symbols · green dots: received data"; font.pixelSize: 10; Layout.fillWidth: true; wrapMode: Text.WordWrap }
                    }
                }
                    SnrPlot {
                        Layout.fillWidth: true
                        Layout.minimumWidth: 0
                        active: modeTabs.currentIndex === 0 && plotTabs.currentIndex === 2
                        estimates: receiver.rbSnr
                        running: receiver.running
                    }
                }
            }

            Card {
                Layout.fillWidth: true
                Layout.leftMargin: window.pageMargin
                Layout.rightMargin: window.pageMargin
                implicitHeight: health.implicitHeight + 28
                GridLayout {
                    id: health
                    anchors.fill: parent
                    anchors.margins: 14
                    columns: window.compact ? 1 : window.width < 1000 ? 2 : 4
                    columnSpacing: 24
                    rowSpacing: 8
                    Caption {
                        text: "Input RMS  " + (receiver.running ? receiver.inputLevel.toFixed(1) + " dBFS" : "—")
                    }
                    Label {
                        text: "Dropped frames  " + receiver.droppedFrames
                        color: receiver.droppedFrames > 0 ? "#ffc078" : window.muted
                        font.pixelSize: 12
                    }
                    Label {
                        text: "Clipped samples  " + receiver.clippedSamples
                        color: receiver.clippedSamples > 0 ? "#ffc078" : window.muted
                        font.pixelSize: 12
                    }
                    Caption { text: "Statistics reset when a new session starts"; Layout.fillWidth: true; wrapMode: Text.WordWrap }
                }
            }
            Item { Layout.preferredHeight: 6 }
        }
    }
}
