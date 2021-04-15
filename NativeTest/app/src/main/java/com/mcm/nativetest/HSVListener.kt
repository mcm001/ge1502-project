package com.mcm.nativetest

import android.widget.SeekBar
import android.widget.SeekBar.OnSeekBarChangeListener
import android.widget.TextView
import org.photonvision.common.util.numbers.DoubleCouple
import org.photonvision.common.util.numbers.IntegerCouple
import org.photonvision.common.util.numbers.NumberCouple
import org.photonvision.vision.opencv.ContourShape
import org.photonvision.vision.pipeline.ColoredShapePipelineSettings

class HSVListener(private var mainActivity: MainActivity) : OnSeekBarChangeListener {
    private var hueMinText: TextView = mainActivity.findViewById(R.id.hueMinText)
    private var hueMaxText: TextView = mainActivity.findViewById(R.id.hueMaxText)
    private var hueMin: SeekBar = mainActivity.findViewById(R.id.hueMin)
    private var hueMax: SeekBar = mainActivity.findViewById(R.id.hueMax)
    private var satMinText: TextView = mainActivity.findViewById(R.id.satMinText)
    private var satMaxText: TextView = mainActivity.findViewById(R.id.satMaxText)
    private var satMin: SeekBar = mainActivity.findViewById(R.id.satMin)
    private var satMax: SeekBar = mainActivity.findViewById(R.id.satMax)
    private var valueMinText: TextView = mainActivity.findViewById(R.id.valueMinText)
    private var valueMaxText: TextView = mainActivity.findViewById(R.id.valueMaxText)
    private var valueMin: SeekBar = mainActivity.findViewById(R.id.valueMin)
    private var valueMax: SeekBar = mainActivity.findViewById(R.id.valueMax)
    private var accuracyText: TextView = mainActivity.findViewById(R.id.accuracyText)
    private var accuracy: SeekBar = mainActivity.findViewById(R.id.accuracy)

    init {
        // Hue
        hueMin.setOnSeekBarChangeListener(this)
        hueMax.setOnSeekBarChangeListener(this)

        satMin.setOnSeekBarChangeListener(this)
        satMax.setOnSeekBarChangeListener(this)

        valueMin.setOnSeekBarChangeListener(this)
        valueMax.setOnSeekBarChangeListener(this)

        // Accuracy percentage
        accuracy.setOnSeekBarChangeListener(this)
    }

    fun init() {
        // Set initial settings
        val thread = mainActivity.visionProcess
        val settings = thread?.pipelineSettings
        if (settings != null) {
            settings.accuracyPercentage = 75.0
            settings.circleAccuracy = 12
            settings.hsvHue = IntegerCouple(64, 170)
            settings.hsvSaturation = IntegerCouple(199, 255)
            settings.hsvValue = IntegerCouple(179, 255)
            settings.contourArea = DoubleCouple(1.0 / 100.0, 100)
            settings.contourShape = ContourShape.Circle

            setCouple(hueMin, hueMax, settings.hsvHue)
            setCouple(satMin, satMax, settings.hsvSaturation)
            setCouple(valueMin, valueMax, settings.hsvValue)
            accuracy.progress = settings.accuracyPercentage.toInt()

            updateText(settings)
        }
    }

    private fun <T : Number> setCouple(lower: SeekBar, upper: SeekBar, value: NumberCouple<T>) {
        lower.progress = value.first.toInt()
        upper.progress = value.second.toInt()
    }

    override fun onProgressChanged(seekBar: SeekBar, progress: Int, fromUser: Boolean) {
        val thread: VisionProcessThread = mainActivity.visionProcess ?: return
        if (!fromUser) return

        val settings = thread.pipelineSettings
        updateSettings(settings)
        updateText(settings)

        thread.setSettings(settings)
    }

    private fun updateSettings(settings: ColoredShapePipelineSettings) {
        settings.hsvHue = IntegerCouple(hueMin.progress, hueMax.progress)
        settings.hsvSaturation = IntegerCouple(satMin.progress, satMax.progress)
        settings.hsvValue = IntegerCouple(valueMin.progress, valueMax.progress)
        settings.accuracyPercentage = accuracy.progress.toDouble()

        // hue
//        if (settings.hsvHue.first > settings.hsvHue.second) {
//            val lower = settings.hsvHue.second
//            val upper = settings.hsvHue.first
//            settings.hsvHue = IntegerCouple(lower, upper)
//            hueMin.progress = lower
//            hueMax.progress = upper
//        }

        // sat
        if (settings.hsvSaturation.first > settings.hsvSaturation.second) {
            val lower = settings.hsvSaturation.second
            val upper = settings.hsvSaturation.first
            settings.hsvSaturation = IntegerCouple(lower, upper)
            satMin.progress = lower
            satMax.progress = upper
        }

        // value
        if (settings.hsvValue.first > settings.hsvValue.second) {
            val lower = settings.hsvValue.second
            val upper = settings.hsvValue.first
            settings.hsvValue = IntegerCouple(lower, upper)
            valueMin.progress = lower
            valueMax.progress = upper
        }
    }

    private fun updateText(settings: ColoredShapePipelineSettings) {
        accuracyText.text = "Accuracy: ${settings.accuracyPercentage}"
        hueMinText.text = "Hue Min: " + settings.hsvHue.first
        hueMaxText.text = "Hue Max: " + settings.hsvHue.second
        satMinText.text = "Sat Min: " + settings.hsvSaturation.first
        satMaxText.text = "Sat Max: " + settings.hsvSaturation.second
        valueMinText.text = "Value Min: " + settings.hsvValue.first
        valueMaxText.text = "Value Max: " + settings.hsvValue.second
    }

    override fun onStartTrackingTouch(seekBar: SeekBar) {}
    override fun onStopTrackingTouch(seekBar: SeekBar) {}
}