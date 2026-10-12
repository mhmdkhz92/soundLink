import QtQuick
import QtQuick.Layouts
import QtQuick.Controls.Basic

Rectangle {
    id: plot
    property bool active: false
    property bool running: false
    property var estimates: []
    property int selectedRb: -1
    color: "#131f2e"
    border.color: "#26364a"
    radius: 14
    onActiveChanged: if (active) canvas.requestPaint()
    onEstimatesChanged: if (active) canvas.requestPaint()

    function selectedText() {
        if (selectedRb < 0 || selectedRb >= estimates.length)
            return "Tap a bar to inspect its estimate"
        const rb = estimates[selectedRb]
        return "RB " + rb.rb + " · " + (rb.valid ? (rb.capped ? "≥ " : "") + rb.db.toFixed(1) + " dB" : "Unavailable")
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 18
        spacing: 8
        RowLayout {
            Layout.fillWidth: true
            Label { text: "SNR per RB"; font.pixelSize: 17; font.weight: Font.DemiBold; color: "#e4edf6" }
            Item { Layout.fillWidth: true }
            Label { text: "dB"; font.pixelSize: 12; color: "#91a5ba" }
        }
        Canvas {
            id: canvas
            Layout.fillWidth: true
            Layout.fillHeight: true
            onWidthChanged: if (plot.active) requestPaint()
            onHeightChanged: if (plot.active) requestPaint()
            onPaint: {
                if (!plot.active || !visible) return
                const ctx = getContext("2d")
                ctx.clearRect(0, 0, width, height)
                const left = 34, top = 12, right = width - 10, bottom = height - 30
                const w = right - left, h = bottom - top
                if (w <= 0 || h <= 0) return
                const y = function(db) { return bottom - (Math.max(-10, Math.min(60, db)) + 10) / 70 * h }
                ctx.font = "10px sans-serif"
                ctx.lineWidth = 1
                for (let db = -10; db <= 60; db += 10) {
                    ctx.strokeStyle = db === 0 ? "#52657a" : "#293b4e"
                    ctx.beginPath(); ctx.moveTo(left, y(db)); ctx.lineTo(right, y(db)); ctx.stroke()
                    ctx.fillStyle = "#91a5ba"; ctx.textAlign = "right"
                    ctx.fillText(String(db), left - 6, y(db) + 3)
                }
                const rbs = plot.estimates
                let validCount = 0
                if (rbs.length) {
                    const pitch = w / rbs.length
                    const gap = Math.min(3, pitch * 0.2)
                    const tickStep = Math.max(1, Math.ceil(rbs.length / 6))
                    for (let i = 0; i < rbs.length; ++i) {
                        const x = left + i * pitch
                        if (rbs[i].valid) {
                            ++validCount
                            ctx.fillStyle = rbs[i].db < 0 ? "#ffc078" : "#39d6b2"
                            ctx.fillRect(x + gap / 2, y(rbs[i].db), pitch - gap, bottom - y(rbs[i].db))
                        } else {
                            ctx.fillStyle = "#52657a"
                            ctx.fillRect(x + gap / 2, bottom - 3, pitch - gap, 3)
                        }
                        if (i === 0 || i === rbs.length - 1 || (i + 1) % tickStep === 0) {
                            ctx.fillStyle = "#91a5ba"; ctx.textAlign = "center"
                            ctx.fillText(String(rbs[i].rb), x + pitch / 2, bottom + 15)
                        }
                    }
                    ctx.fillStyle = "#91a5ba"; ctx.textAlign = "center"
                    ctx.fillText("Resource block", left + w / 2, height - 1)
                }
                if (!validCount) {
                    ctx.fillStyle = "#b6c5d4"; ctx.textAlign = "center"; ctx.font = "12px sans-serif"
                    ctx.fillText("Waiting for channel estimates", left + w / 2, top + h / 2)
                }
            }
            MouseArea {
                anchors.fill: parent
                onClicked: function(mouse) {
                    const w = canvas.width - 44
                    plot.selectedRb = mouse.x >= 34 && mouse.x < canvas.width - 10 && w > 0
                        ? Math.floor((mouse.x - 34) / w * plot.estimates.length) : -1
                }
            }
        }
        Label { text: plot.selectedText(); color: "#b6c5d4"; font.pixelSize: 12; Layout.fillWidth: true; wrapMode: Text.WordWrap }
        Label { text: "Post-EQ estimate · 10 Hz · gray: unavailable · capped at 60 dB"; color: "#91a5ba"; font.pixelSize: 10; Layout.fillWidth: true; wrapMode: Text.WordWrap }
    }
}
